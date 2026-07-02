use std::time::{Duration, Instant};

use crate::command::NavMode;
use crate::mapping::Frontier;
use crate::odometry::Pose;
use crate::slam::Slam;

const REPLAN_INTERVAL: Duration = Duration::from_millis(1000);
const BLACKLIST_TTL: Duration = Duration::from_secs(45);
const PROGRESS_TIMEOUT: Duration = Duration::from_secs(8);
/// Translation (m) / rotation (rad) below which the robot counts as not moving.
const PROGRESS_MIN_DIST: f32 = 0.03;
const PROGRESS_MIN_ANGLE: f32 = 0.08;
const GOAL_TOLERANCE: f32 = 0.20;
/// Frontiers closer than this are skipped to prevent tiny local oscillations.
const MIN_FRONTIER_DIST: f32 = 0.6;
/// Frontiers within this radius of a blacklisted goal are also skipped.
const BLACKLIST_RADIUS: f32 = 0.8;
const LOOKAHEAD_DIST: f32 = 0.25;
/// Front obstacle distance below which the robot rotates in place.
const OBSTACLE_STOP_DIST: f32 = 0.35;

/// Actions the navigator asks the session to perform. The navigator itself
/// never touches sockets, channels or the GUI — it only produces these.
#[derive(Debug, Clone, PartialEq)]
pub enum NavUpdate {
    Drive {
        left_power: u8,
        left_angle: f32,
        right_power: u8,
        right_angle: f32,
    },
    /// Stop the motors.
    Stop,
    PathChanged(Vec<(f32, f32)>),
    GoalChanged(Option<(f32, f32)>),
    /// Fresh frontier set computed during an exploration replan.
    FrontiersComputed(Vec<Frontier>),
    /// A NavigateTo goal was reached or abandoned; mode fell back to Manual.
    Finished { success: bool },
}

/// Explicit navigation state machine. Consumes mode changes and pose/telemetry
/// ticks, produces [`NavUpdate`] actions.
pub struct Navigator {
    mode: NavMode,
    path: Vec<(f32, f32)>,
    goal: Option<(f32, f32)>,
    last_replan: Instant,
    last_progress_time: Instant,
    last_progress_pose: Pose,
    blacklist: Vec<(f32, f32, Instant)>,
}

impl Navigator {
    pub fn new() -> Self {
        Self {
            mode: NavMode::Manual,
            path: Vec::new(),
            goal: None,
            last_replan: Instant::now() - REPLAN_INTERVAL * 10,
            last_progress_time: Instant::now(),
            last_progress_pose: Pose { x: 0.0, y: 0.0, theta: 0.0 },
            blacklist: Vec::new(),
        }
    }

    pub fn set_mode(&mut self, mode: NavMode, pose: Pose) -> Vec<NavUpdate> {
        if mode == self.mode {
            return Vec::new();
        }
        log::info!("[NAV] Mode change: {:?} -> {:?}", self.mode, mode);
        let leaving_autonomous = self.mode != NavMode::Manual;
        self.mode = mode;
        self.path.clear();
        self.goal = None;
        self.last_replan = Instant::now() - REPLAN_INTERVAL * 10; // force immediate replan
        self.last_progress_time = Instant::now();
        self.last_progress_pose = pose;

        let mut updates = vec![NavUpdate::PathChanged(Vec::new()), NavUpdate::GoalChanged(None)];
        if mode == NavMode::Manual && leaving_autonomous {
            // Autonomous control may have left the motors running.
            updates.push(NavUpdate::Stop);
        }
        updates
    }

    /// Advance the state machine. Called on every telemetry event and on
    /// periodic ticks (so stuck detection runs even if telemetry stalls).
    pub fn tick(&mut self, slam: &dyn Slam, pose: Pose, min_front_dist: f32) -> Vec<NavUpdate> {
        if self.mode == NavMode::Manual {
            return Vec::new();
        }

        let mut updates = Vec::new();

        if self.path.is_empty() || self.last_replan.elapsed() >= REPLAN_INTERVAL {
            self.replan(slam, pose, &mut updates);
            self.last_replan = Instant::now();
        }

        self.check_stuck(pose, &mut updates);
        self.check_goal_reached(pose, &mut updates);

        if !self.path.is_empty() {
            updates.push(self.follow_path(pose, min_front_dist));
        }

        updates
    }

    fn replan(&mut self, slam: &dyn Slam, pose: Pose, updates: &mut Vec<NavUpdate>) {
        self.blacklist.retain(|(_, _, t)| t.elapsed() < BLACKLIST_TTL);

        let target = match self.mode {
            NavMode::Manual => return,
            NavMode::NavigateTo { x, y } => Some((x, y)),
            NavMode::Exploration => {
                let frontiers = slam.get_frontiers();
                updates.push(NavUpdate::FrontiersComputed(frontiers.clone()));

                // Goal persistence: keep the current frontier target until it is
                // reached, becomes unreachable, or times out — only then pick a
                // new one. Prevents flip-flopping between frontiers every replan.
                self.goal.or_else(|| {
                    frontiers
                        .iter()
                        .filter(|f| {
                            let dist = ((f.centroid_x - pose.x).powi(2) + (f.centroid_y - pose.y).powi(2)).sqrt();
                            dist >= MIN_FRONTIER_DIST && !self.is_blacklisted(f.centroid_x, f.centroid_y)
                        })
                        .max_by_key(|f| f.size)
                        .map(|best| {
                            log::info!(
                                "[NAV] Exploration: selected frontier target X={:.2}, Y={:.2}",
                                best.centroid_x, best.centroid_y
                            );
                            (best.centroid_x, best.centroid_y)
                        })
                })
            }
        };

        let Some((gx, gy)) = target else {
            log::info!("[NAV] Exploration: no reachable frontiers found.");
            self.abandon_goal(updates);
            return;
        };

        match slam.plan_path(gx, gy) {
            Some(path) => {
                self.path = path;
                self.goal = Some((gx, gy));
                self.last_progress_time = Instant::now();
                self.last_progress_pose = pose;
                updates.push(NavUpdate::PathChanged(self.path.clone()));
                updates.push(NavUpdate::GoalChanged(self.goal));
            }
            None => {
                log::info!("[NAV] A* failed to find path to X={:.2}, Y={:.2}.", gx, gy);
                match self.mode {
                    NavMode::Exploration => {
                        self.blacklist.push((gx, gy, Instant::now()));
                        self.abandon_goal(updates);
                    }
                    NavMode::NavigateTo { .. } => self.finish(false, updates),
                    NavMode::Manual => {}
                }
            }
        }
    }

    fn check_stuck(&mut self, pose: Pose, updates: &mut Vec<NavUpdate>) {
        let Some(goal) = self.goal else { return };
        if self.path.is_empty() {
            return;
        }

        let dist_moved = ((pose.x - self.last_progress_pose.x).powi(2)
            + (pose.y - self.last_progress_pose.y).powi(2))
        .sqrt();
        let angle_moved = (pose.theta - self.last_progress_pose.theta).abs();
        if dist_moved > PROGRESS_MIN_DIST || angle_moved > PROGRESS_MIN_ANGLE {
            self.last_progress_time = Instant::now();
            self.last_progress_pose = pose;
            return;
        }

        if self.last_progress_time.elapsed() < PROGRESS_TIMEOUT {
            return;
        }

        match self.mode {
            NavMode::Exploration => {
                log::warn!(
                    "[NAV] Stuck: no progress for {:?}. Blacklisting frontier X={:.2}, Y={:.2}",
                    PROGRESS_TIMEOUT, goal.0, goal.1
                );
                self.blacklist.push((goal.0, goal.1, Instant::now()));
                self.abandon_goal(updates);
            }
            NavMode::NavigateTo { .. } => {
                log::warn!(
                    "[NAV] Stuck: target X={:.2}, Y={:.2} unreachable. Cancelling navigation.",
                    goal.0, goal.1
                );
                self.finish(false, updates);
            }
            NavMode::Manual => {}
        }
    }

    fn check_goal_reached(&mut self, pose: Pose, updates: &mut Vec<NavUpdate>) {
        let Some(goal) = self.goal else { return };
        if self.path.is_empty() {
            return;
        }
        let dist = ((goal.0 - pose.x).powi(2) + (goal.1 - pose.y).powi(2)).sqrt();
        if dist >= GOAL_TOLERANCE {
            return;
        }
        log::info!("[NAV] Goal reached within {:.2}m.", dist);
        match self.mode {
            // Exploration continues: drop this goal and pick a new frontier on
            // the next replan.
            NavMode::Exploration => self.abandon_goal(updates),
            NavMode::NavigateTo { .. } => self.finish(true, updates),
            NavMode::Manual => {}
        }
    }

    /// Pure-pursuit style path following with obstacle-triggered rotation.
    fn follow_path(&self, pose: Pose, min_front_dist: f32) -> NavUpdate {
        let target = self
            .path
            .iter()
            .find(|(wx, wy)| {
                let dist = ((wx - pose.x).powi(2) + (wy - pose.y).powi(2)).sqrt();
                dist > LOOKAHEAD_DIST
            })
            .or_else(|| self.path.last())
            .copied()
            .expect("follow_path called with non-empty path");

        let target_theta = (target.1 - pose.y).atan2(target.0 - pose.x);
        let mut angle_diff = target_theta - pose.theta;
        while angle_diff > std::f32::consts::PI {
            angle_diff -= 2.0 * std::f32::consts::PI;
        }
        while angle_diff < -std::f32::consts::PI {
            angle_diff += 2.0 * std::f32::consts::PI;
        }

        if min_front_dist < OBSTACLE_STOP_DIST || angle_diff.abs() > 0.4 {
            // Rotate in place: toward the waypoint, or away from the obstacle.
            if angle_diff > 0.0 {
                NavUpdate::Drive { left_power: 120, left_angle: -1.0, right_power: 120, right_angle: 1.0 }
            } else {
                NavUpdate::Drive { left_power: 120, left_angle: 1.0, right_power: 120, right_angle: -1.0 }
            }
        } else {
            // Blend forward motion with a proportional turn correction.
            let base_pwr = 100;
            let turn = (angle_diff * 50.0) as i32;
            let left = (base_pwr - turn).clamp(40, 150) as u8;
            let right = (base_pwr + turn).clamp(40, 150) as u8;
            NavUpdate::Drive { left_power: left, left_angle: 1.0, right_power: right, right_angle: 1.0 }
        }
    }

    fn is_blacklisted(&self, x: f32, y: f32) -> bool {
        self.blacklist.iter().any(|(bx, by, _)| {
            let dx = x - bx;
            let dy = y - by;
            (dx * dx + dy * dy).sqrt() < BLACKLIST_RADIUS
        })
    }

    /// Drop the current goal but stay in the current mode (exploration picks a
    /// new frontier on the next replan).
    fn abandon_goal(&mut self, updates: &mut Vec<NavUpdate>) {
        let had_path = !self.path.is_empty();
        self.path.clear();
        self.goal = None;
        updates.push(NavUpdate::PathChanged(Vec::new()));
        updates.push(NavUpdate::GoalChanged(None));
        if had_path {
            updates.push(NavUpdate::Stop);
        }
    }

    /// End a NavigateTo mission and fall back to Manual.
    fn finish(&mut self, success: bool, updates: &mut Vec<NavUpdate>) {
        self.mode = NavMode::Manual;
        self.path.clear();
        self.goal = None;
        updates.push(NavUpdate::PathChanged(Vec::new()));
        updates.push(NavUpdate::GoalChanged(None));
        updates.push(NavUpdate::Stop);
        updates.push(NavUpdate::Finished { success });
    }

    pub fn reset(&mut self) {
        *self = Navigator::new();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::homerobot::LidarPoint;
    use std::time::Instant;

    struct StubSlam {
        frontiers: Vec<Frontier>,
        path: Option<Vec<(f32, f32)>>,
    }

    impl Slam for StubSlam {
        fn update(&mut self, _scan: &[LidarPoint], odom_pose: &Pose) -> Pose {
            *odom_pose
        }
        fn add_odom_pose(&mut self, _pose: Pose, _timestamp: Instant) {}
        fn get_map_data(&self) -> (usize, usize, &[i16]) {
            (0, 0, &[])
        }
        fn save_map(&self, _filename: &str) -> std::io::Result<()> {
            Ok(())
        }
        fn get_frontiers(&self) -> Vec<Frontier> {
            self.frontiers.clone()
        }
        fn plan_path(&self, _goal_x: f32, _goal_y: f32) -> Option<Vec<(f32, f32)>> {
            self.path.clone()
        }
    }

    fn pose(x: f32, y: f32, theta: f32) -> Pose {
        Pose { x, y, theta }
    }

    fn frontier(x: f32, y: f32, size: usize) -> Frontier {
        Frontier { centroid_x: x, centroid_y: y, size }
    }

    #[test]
    fn navigate_to_plans_path_and_drives_toward_goal() {
        let mut nav = Navigator::new();
        let slam = StubSlam { frontiers: vec![], path: Some(vec![(0.5, 0.0), (1.0, 0.0)]) };
        let p = pose(0.0, 0.0, 0.0);

        nav.set_mode(NavMode::NavigateTo { x: 1.0, y: 0.0 }, p);
        let updates = nav.tick(&slam, p, 10.0);

        assert!(updates.contains(&NavUpdate::PathChanged(vec![(0.5, 0.0), (1.0, 0.0)])));
        assert!(updates.contains(&NavUpdate::GoalChanged(Some((1.0, 0.0)))));
        // Goal straight ahead: both wheels forward.
        assert!(matches!(
            updates.last(),
            Some(NavUpdate::Drive { left_angle, right_angle, .. })
                if *left_angle == 1.0 && *right_angle == 1.0
        ));
    }

    #[test]
    fn unreachable_navigate_to_finishes_and_falls_back_to_manual() {
        let mut nav = Navigator::new();
        let slam = StubSlam { frontiers: vec![], path: None };
        let p = pose(0.0, 0.0, 0.0);

        nav.set_mode(NavMode::NavigateTo { x: 3.0, y: 0.0 }, p);
        let updates = nav.tick(&slam, p, 10.0);

        assert!(updates.contains(&NavUpdate::Stop));
        assert!(updates.contains(&NavUpdate::Finished { success: false }));
        // Back in Manual: further ticks are inert.
        assert!(nav.tick(&slam, p, 10.0).is_empty());
    }

    #[test]
    fn reaching_navigate_to_goal_stops_and_finishes() {
        let mut nav = Navigator::new();
        let slam = StubSlam { frontiers: vec![], path: Some(vec![(1.0, 0.0)]) };
        let near_goal = pose(0.9, 0.0, 0.0);

        nav.set_mode(NavMode::NavigateTo { x: 1.0, y: 0.0 }, near_goal);
        let updates = nav.tick(&slam, near_goal, 10.0);

        assert!(updates.contains(&NavUpdate::Stop));
        assert!(updates.contains(&NavUpdate::Finished { success: true }));
    }

    #[test]
    fn exploration_skips_frontiers_too_close_to_the_robot() {
        let mut nav = Navigator::new();
        let slam = StubSlam {
            // The near frontier is bigger but within MIN_FRONTIER_DIST.
            frontiers: vec![frontier(0.3, 0.0, 100), frontier(2.0, 0.0, 10)],
            path: Some(vec![(2.0, 0.0)]),
        };
        let p = pose(0.0, 0.0, 0.0);

        nav.set_mode(NavMode::Exploration, p);
        let updates = nav.tick(&slam, p, 10.0);

        assert!(updates.contains(&NavUpdate::GoalChanged(Some((2.0, 0.0)))));
    }

    #[test]
    fn exploration_continues_after_reaching_a_frontier() {
        let mut nav = Navigator::new();
        // Biggest frontier gets picked first; the other is the follow-up.
        let slam = StubSlam {
            frontiers: vec![frontier(2.0, 0.0, 100), frontier(5.0, 0.0, 50)],
            path: Some(vec![(2.0, 0.0)]),
        };

        let start = pose(0.0, 0.0, 0.0);
        nav.set_mode(NavMode::Exploration, start);
        let t1 = nav.tick(&slam, start, 10.0);
        assert!(t1.contains(&NavUpdate::GoalChanged(Some((2.0, 0.0)))));

        // Arriving at the frontier drops the goal but stays in Exploration.
        let arrived = pose(2.0, 0.0, 0.0);
        let t2 = nav.tick(&slam, arrived, 10.0);
        assert!(t2.contains(&NavUpdate::GoalChanged(None)));
        assert!(t2.contains(&NavUpdate::Stop));
        assert!(!t2.iter().any(|u| matches!(u, NavUpdate::Finished { .. })));

        // Next tick replans immediately (path empty) and picks the next
        // frontier; the reached one is now too close to be a candidate.
        let t3 = nav.tick(&slam, arrived, 10.0);
        assert!(t3.contains(&NavUpdate::GoalChanged(Some((5.0, 0.0)))));
    }

    #[test]
    fn obstacle_ahead_forces_rotation_in_place() {
        let mut nav = Navigator::new();
        let slam = StubSlam { frontiers: vec![], path: Some(vec![(2.0, 0.0)]) };
        let p = pose(0.0, 0.0, 0.0);

        nav.set_mode(NavMode::NavigateTo { x: 2.0, y: 0.0 }, p);
        let updates = nav.tick(&slam, p, 0.2); // obstacle at 20cm

        assert!(matches!(
            updates.last(),
            Some(NavUpdate::Drive { left_angle, right_angle, .. })
                if *left_angle == -*right_angle
        ));
    }

    #[test]
    fn manual_mode_change_stops_autonomous_motion() {
        let mut nav = Navigator::new();
        let p = pose(0.0, 0.0, 0.0);
        nav.set_mode(NavMode::Exploration, p);
        let updates = nav.set_mode(NavMode::Manual, p);
        assert!(updates.contains(&NavUpdate::Stop));
    }
}
