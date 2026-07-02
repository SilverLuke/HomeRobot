//! Persistent world model: the SLAM map and pose estimate, owned by `main`
//! and shared with robot sessions. A reconnecting robot resumes with the
//! existing map instead of remapping from scratch.

use crate::odometry::Pose;
use crate::slam::BasicSlam;

pub struct WorldModel {
    pub slam: BasicSlam,
    /// Best pose estimate: SLAM-corrected on each sweep, dead-reckoned from
    /// encoder deltas in between.
    pub pose: Pose,
    pub pose_initialized: bool,
}

impl WorldModel {
    pub fn new() -> Self {
        Self {
            slam: BasicSlam::new(),
            pose: Pose { x: 0.0, y: 0.0, theta: 0.0 },
            pose_initialized: false,
        }
    }

    /// Called when a robot (re)connects. The server-side odometry restarts at
    /// zero for each connection, so SLAM must drop its odometry-frame tracking
    /// state — the map and the pose estimate survive.
    pub fn on_new_session(&mut self) {
        self.slam.on_odometry_reset();
    }

    pub fn reset(&mut self) {
        *self = WorldModel::new();
    }
}
