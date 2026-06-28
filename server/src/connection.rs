use std::net::TcpStream;
use std::sync::{Arc, Mutex, mpsc};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};
use std::thread::sleep;
use std::io;

use crate::stats::Stats;
use crate::command::RobotCommand;
use crate::gui::GuiUpdate;
use crate::reader;
use crate::slam::{Slam, BasicSlam};
use crate::sender::send_manual_command;
use crate::homerobot::robot_to_server_message::Payload;

lazy_static::lazy_static! {
    pub static ref MOTION_COMPLETED: Arc<(Mutex<Option<Result<(), String>>>, std::sync::Condvar)> = 
        Arc::new((Mutex::new(None), std::sync::Condvar::new()));
}

/// Handles a single robot connection
pub fn handle_connection(
    stream: TcpStream, 
    robot_command: Arc<Mutex<RobotCommand>>, 
    stats: Arc<Stats>, 
    sig_count: Arc<AtomicUsize>, 
    gui_tx: mpsc::Sender<GuiUpdate>,
    rec: Arc<Mutex<rerun::RecordingStream>>
) {
    stats.active_connections.fetch_add(1, Ordering::SeqCst);
    let addr = stream.peer_addr().unwrap_or_else(|_| "unknown".parse().unwrap());
    stats.log(&format!("[CONN] New connection from {}", addr));
    let _ = gui_tx.send(GuiUpdate::Status(format!("Connected: {}", addr)));
    
    stream.set_nonblocking(true).ok();
    stream.set_read_timeout(Some(Duration::from_secs(5))).ok();

    let mut protocol = reader::ProtocolManager::new(stream, stats.clone());
    let mut odom = crate::odometry::Odometry::new();
    let mut slam = BasicSlam::new();
    let start_time = Instant::now();
    let mut last_sent_command = RobotCommand::default();
    let mut min_front_dist = 10.0_f32;

    // Scan buffering: accumulate points until scan_completed marks a full revolution.
    let mut pending_scan: Vec<crate::homerobot::LidarPoint> = Vec::new();
    let mut last_scan_flush_time = Instant::now() - Duration::from_secs(5);
    // Scan-rate tracking: timestamps of the last N completed revolutions.
    let mut revolution_times: std::collections::VecDeque<Instant> = std::collections::VecDeque::new();

    // Lidar 5s summary stats
    let mut last_lidar_summary_time = Instant::now();
    let mut summary_sweeps = 0;
    let mut summary_total_points = 0;
    let mut summary_sum_avg_delta = 0.0;
    let mut summary_sum_std_dev = 0.0;
    let mut summary_min_delta = f32::MAX;
    let mut summary_max_delta = f32::MIN;

    let mut current_pose = crate::odometry::Pose { x: 0.0, y: 0.0, theta: 0.0 };
    let mut current_pose_initialized = false;
    let mut active_path: Vec<(f32, f32)> = Vec::new();
    let mut navigation_goal: Option<(f32, f32)> = None;
    let mut last_replan_time = Instant::now() - Duration::from_secs(10); // force immediate replan on start

    // Progress and blacklist tracking to handle unreachable targets/frontiers
    let mut last_progress_time = Instant::now();
    let mut last_progress_pose = crate::odometry::Pose { x: 0.0, y: 0.0, theta: 0.0 };
    let mut blacklisted_frontiers: Vec<(f32, f32, Instant)> = Vec::new();

    #[derive(Clone, PartialEq, Debug)]
    enum NavMode {
        None,
        Exploration,
        NavigateTo(f32, f32),
    }
    let mut last_mode = NavMode::None;
    
    while stats.running.load(Ordering::Relaxed) && sig_count.load(Ordering::Relaxed) == 0 {
        let elapsed = start_time.elapsed().as_secs_f64();
        let rec = rec.lock().unwrap().clone();
        rec.set_time("realtime", std::time::Duration::from_secs_f64(elapsed));

        let mut telemetry_received = false;

        loop {
            match protocol.read_message() {
                Ok(Some(msg)) => {
                    if let Some(payload) = msg.payload {
                        match payload {
                            Payload::Battery(bat) => {
                                log::info!("[SERVER] Telemetry Heartbeat: Battery {}%", bat.percentage);
                                let _ = gui_tx.send(GuiUpdate::Battery { percentage: bat.percentage, voltage_mv: bat.voltage_mv });
                                rec.log("robot/battery", &rerun::Scalars::new([bat.percentage as f64])).ok();
                            }
                            Payload::Encoders(enc) => {
                                telemetry_received = true;
                                if enc.left_encoder != 0 || enc.right_encoder != 0 {
                                    log::info!("[SERVER] Telemetry Heartbeat: Encoders delta L={} R={}", enc.left_encoder, enc.right_encoder);
                                }
                                let old_odom_pose = odom.pose;
                                odom.update_encoders(enc.left_encoder, enc.right_encoder, msg.sequence_millis);
                                slam.add_odom_pose(odom.pose, Instant::now());
                                if !current_pose_initialized {
                                    current_pose = odom.pose;
                                    current_pose_initialized = true;
                                } else {
                                    // Propagate the odometry change to current_pose so the path-following controller 
                                    // doesn't operate on stale position information between slower SLAM sweeps.
                                    let dx = odom.pose.x - old_odom_pose.x;
                                    let dy = odom.pose.y - old_odom_pose.y;
                                    let dt = odom.pose.theta - old_odom_pose.theta;

                                    // Rotate odometry translation delta to align with the current SLAM pose orientation
                                    let diff_theta = current_pose.theta - old_odom_pose.theta;
                                    let cos_diff = diff_theta.cos();
                                    let sin_diff = diff_theta.sin();
                                    let dx_map = dx * cos_diff - dy * sin_diff;
                                    let dy_map = dx * sin_diff + dy * cos_diff;

                                    current_pose.x += dx_map;
                                    current_pose.y += dy_map;
                                    current_pose.theta += dt;
                                    while current_pose.theta > std::f32::consts::PI { current_pose.theta -= 2.0 * std::f32::consts::PI; }
                                    while current_pose.theta < -std::f32::consts::PI { current_pose.theta += 2.0 * std::f32::consts::PI; }

                                    // Send real-time smooth SLAM-based pose updates to the GUI
                                    let _ = gui_tx.send(GuiUpdate::SlamPose {
                                        x: current_pose.x,
                                        y: current_pose.y,
                                        theta: current_pose.theta,
                                    });

                                    // Log dead-reckoned SLAM Pose to Rerun
                                    rec.log("robot/pose/slam", &rerun::Transform3D::from_translation_rotation(
                                        [current_pose.x, current_pose.y, 0.0],
                                        rerun::RotationAxisAngle::new([0.0, 0.0, 1.0], rerun::Angle::from_radians(current_pose.theta))
                                    )).ok();
                                }
                                let _ = gui_tx.send(GuiUpdate::Encoders { left: odom.cumulative_left, right: odom.cumulative_right });
                                let _ = gui_tx.send(GuiUpdate::Pose { 
                                    x: odom.pose.x, 
                                    y: odom.pose.y, 
                                    theta: odom.pose.theta 
                                });

                                // Log Odometry Pose to Rerun
                                rec.log("robot/pose/odom", &rerun::Transform3D::from_translation_rotation(
                                    [odom.pose.x, odom.pose.y, 0.0],
                                    rerun::RotationAxisAngle::new([0.0, 0.0, 1.0], rerun::Angle::from_radians(odom.pose.theta))
                                    )).ok();
                            }
                            Payload::Imu(imu) => {
                                telemetry_received = true;
                                if let (Some(a), Some(g)) = (imu.acceleration, imu.gyroscope) {
                                    odom.update_imu(g.z, msg.sequence_millis);

                                    let (mx, my, mz) = if let Some(m) = imu.magnetometer {
                                        (m.x, m.y, m.z)
                                    } else {
                                        (0.0, 0.0, 0.0)
                                    };
                                    let _ = gui_tx.send(GuiUpdate::Imu { 
                                        ax: a.x, ay: a.y, az: a.z,
                                        gx: g.x, gy: g.y, gz: g.z,
                                        mx, my, mz
                                    });

                                    // Log IMU to Rerun plots
                                    rec.log("robot/imu/accel", &rerun::Arrows3D::from_vectors([[a.x, a.y, a.z]])).ok();
                                    rec.log("robot/imu/gyro/z", &rerun::Scalars::new([g.z as f64])).ok();
                                }
                            }
                            Payload::Lidar(scan) => {
                                telemetry_received = true;

                                // Accumulate points into the pending buffer.
                                // When a scan_completed point arrives it marks the START of a new
                                // revolution (RP-Lidar SYNCBIT), so everything accumulated so far
                                // forms one complete 360° sweep — flush it now.
                                for p in scan.points {
                                    if p.scan_completed {
                                        if !pending_scan.is_empty() && pending_scan.len() >= 30 && last_scan_flush_time.elapsed() >= Duration::from_millis(80) {
                                            // --- Full revolution ready: process the buffered sweep ---
                                            last_scan_flush_time = Instant::now();

                                            // Calculate rotation delta statistics (differences between consecutive angles in sorted order)
                                            if pending_scan.len() >= 2 {
                                                let mut angles: Vec<f32> = pending_scan.iter().map(|p| p.angle_deg).collect();
                                                angles.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                                                
                                                let mut deltas = Vec::with_capacity(angles.len());
                                                for i in 0..angles.len() {
                                                    let next_idx = (i + 1) % angles.len();
                                                    let mut diff = if next_idx == 0 {
                                                        (angles[0] + 360.0) - angles[i]
                                                    } else {
                                                        angles[next_idx] - angles[i]
                                                    };
                                                    if diff < 0.0 {
                                                        diff += 360.0;
                                                    }
                                                    deltas.push(diff);
                                                }
                                                
                                                let n = deltas.len() as f32;
                                                let sum: f32 = deltas.iter().sum();
                                                let avg = sum / n;
                                                
                                                let variance: f32 = deltas.iter().map(|&d| {
                                                    let diff = d - avg;
                                                    diff * diff
                                                }).sum::<f32>() / n;
                                                let std_dev = variance.sqrt();
                                                
                                                let mut min_val = f32::MAX;
                                                let mut max_val = f32::MIN;
                                                for &d in &deltas {
                                                    if d < min_val { min_val = d; }
                                                    if d > max_val { max_val = d; }
                                                }
                                                
                                                summary_sweeps += 1;
                                                summary_total_points += angles.len();
                                                summary_sum_avg_delta += avg;
                                                summary_sum_std_dev += std_dev;
                                                if min_val < summary_min_delta { summary_min_delta = min_val; }
                                                if max_val > summary_max_delta { summary_max_delta = max_val; }

                                                if last_lidar_summary_time.elapsed() >= Duration::from_secs(5) {
                                                    if summary_sweeps > 0 {
                                                        let avg_sweep_points = (summary_total_points as f32) / (summary_sweeps as f32);
                                                        let overall_avg_delta = summary_sum_avg_delta / (summary_sweeps as f32);
                                                        let overall_avg_std_dev = summary_sum_std_dev / (summary_sweeps as f32);
                                                        stats.log(&format!(
                                                            "[LIDAR Summary (last 5s)] Received {} updates, total {} points | Sweep points: {:.1}, Avg delta: {:.4}°, StdDev: {:.4}°, Min: {:.4}°, Max: {:.4}°",
                                                            summary_sweeps, summary_total_points, avg_sweep_points, overall_avg_delta, overall_avg_std_dev, summary_min_delta, summary_max_delta
                                                        ));
                                                    }
                                                    summary_sweeps = 0;
                                                    summary_total_points = 0;
                                                    summary_sum_avg_delta = 0.0;
                                                    summary_sum_std_dev = 0.0;
                                                    summary_min_delta = f32::MAX;
                                                    summary_max_delta = f32::MIN;
                                                    last_lidar_summary_time = Instant::now();
                                                }
                                            }

                                            // Track revolution time for scan-rate telemetry.
                                            let now = Instant::now();
                                            revolution_times.push_back(now);
                                            // Keep only the last 5 revolutions for a stable average.
                                            while revolution_times.len() > 5 {
                                                revolution_times.pop_front();
                                            }
                                            // Compute Hz from oldest→newest span.
                                            if revolution_times.len() >= 2 {
                                                let span = revolution_times.back().unwrap()
                                                    .duration_since(*revolution_times.front().unwrap())
                                                    .as_secs_f32();
                                                let hz = (revolution_times.len() as f32 - 1.0) / span;
                                                let _ = gui_tx.send(GuiUpdate::LidarScanRate(hz));
                                            }

                                            // Update obstacle proximity from the complete sweep.
                                            min_front_dist = 10.0;
                                            for pt in &pending_scan {
                                                if pt.angle_deg < 40.0 || pt.angle_deg > 320.0 {
                                                    let d = pt.distance_mm / 1000.0;
                                                    if d > 0.05 && d < min_front_dist {
                                                        min_front_dist = d;
                                                    }
                                                }
                                            }

                                            // Run SLAM on the complete, consistent sweep.
                                            let corrected_pose = slam.update(&pending_scan, &odom.pose);
                                            current_pose = corrected_pose;
                                            current_pose_initialized = true;
                                            let _ = gui_tx.send(GuiUpdate::Lidar(pending_scan.clone()));
                                            let _ = gui_tx.send(GuiUpdate::SlamPose {
                                                x: corrected_pose.x,
                                                y: corrected_pose.y,
                                                theta: corrected_pose.theta,
                                            });

                                            // Log SLAM Pose to Rerun
                                            rec.log("robot/pose/slam", &rerun::Transform3D::from_translation_rotation(
                                                [corrected_pose.x, corrected_pose.y, 0.0],
                                                rerun::RotationAxisAngle::new([0.0, 0.0, 1.0], rerun::Angle::from_radians(corrected_pose.theta))
                                            )).ok();

                                            // Log Lidar Points to Rerun (quality-colored, robot frame)
                                            let (rerun_pts, rerun_colors): (Vec<[f32; 3]>, Vec<rerun::Color>) = pending_scan.iter().map(|pt| {
                                                let angle = -(pt.angle_deg as f32).to_radians();
                                                let d = pt.distance_mm / 1000.0;
                                                let t = (pt.quality as f32 / 15.0).clamp(0.0, 1.0);
                                                let r = ((1.0 - t) * 255.0) as u8;
                                                let g = (t * 255.0) as u8;
                                                ([d * angle.cos(), d * angle.sin(), 0.0], rerun::Color::from_rgb(r, g, 0))
                                            }).unzip();
                                            rec.log("robot/sensor/lidar", &rerun::Points3D::new(rerun_pts).with_colors(rerun_colors)).ok();

                                            // Send map update once per revolution.
                                            let (w, h, data) = slam.get_map_data();
                                            let _ = gui_tx.send(GuiUpdate::Map {
                                                width: w,
                                                height: h,
                                                data: data.to_vec(),
                                            });
                                        }
                                        pending_scan.clear();
                                    }
                                    pending_scan.push(p);
                                }
                            }
                            Payload::Config(conf) => {
                                stats.log("[CONFIG] Robot configuration received");
                                let _ = gui_tx.send(GuiUpdate::Config(conf));
                            }
                            Payload::Capabilities(cap) => {
                                stats.log(&format!(
                                    "[CAPABILITIES] Accel: {}, Gyro: {}, Mag: {}, Wheel Dia: {}mm, Track: {}mm, Ticks/Rev: {}",
                                    cap.has_accelerometer, cap.has_gyroscope, cap.has_magnetometer,
                                    cap.wheel_diameter_mm, cap.wheel_track_mm, cap.encoder_ticks_per_rev
                                ));
                                
                                // 1. Update local odometry model sizes
                                odom.update_sizes(cap.wheel_diameter_mm, cap.wheel_track_mm, cap.encoder_ticks_per_rev);
                                
                                // 2. Update global proxy sizes
                                {
                                    let mut sizes = crate::constants::ROBOT_SIZES.lock().unwrap();
                                    sizes.wheel_base = cap.wheel_track_mm / 1000.0;
                                    sizes.ticks_per_meter = (cap.encoder_ticks_per_rev as f32) / (std::f32::consts::PI * (cap.wheel_diameter_mm / 1000.0));
                                }
                                
                                // 3. Update the GUI thread
                                let _ = gui_tx.send(GuiUpdate::Capabilities {
                                    _has_accelerometer: cap.has_accelerometer,
                                    _has_gyroscope: cap.has_gyroscope,
                                    has_magnetometer: cap.has_magnetometer,
                                    _wheel_diameter_mm: cap.wheel_diameter_mm,
                                    _wheel_track_mm: cap.wheel_track_mm,
                                    _encoder_ticks_per_rev: cap.encoder_ticks_per_rev,
                                });
                            }
                            Payload::RpcResponse(resp) => {
                                stats.log(&format!("[RPC RESPONSE] ID: {}, Error: {}", resp.call_id, resp.error));
                                if resp.error.is_empty() {
                                    stats.log("[RPC] Motion execution completed successfully.");
                                } else {
                                    stats.log(&format!("[RPC] Motion execution ended with error: {}", resp.error));
                                }
                                {
                                    let &(ref lock, ref cvar) = &**MOTION_COMPLETED;
                                    let mut completed = lock.lock().unwrap();
                                    if resp.error.is_empty() {
                                        *completed = Some(Ok(()));
                                    } else {
                                        *completed = Some(Err(resp.error.clone()));
                                    }
                                    cvar.notify_all();
                                }
                                if let Ok(diag_result) = prost::Message::decode(&*resp.payload) {
                                    let diag_result: crate::homerobot::DiagnosticResult = diag_result;
                                    stats.log(&format!("[DIAGNOSTICS] All OK: {}", diag_result.all_ok));
                                    for check in diag_result.checks {
                                        stats.log(&format!("  - {}: {} ({})", check.name, if check.success { "PASS" } else { "FAIL" }, check.message));
                                    }
                                }
                            }
                            _ => {} 
                        }
                    }
                }
                Ok(None) => break,
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) => {
                    stats.log(&format!("[ERROR] Connection to {} lost: {} ({:?})", addr, e, e.kind()));
                    let _ = gui_tx.send(GuiUpdate::Status("Disconnected".to_string()));
                    return; 
                }
            }
        }

        send_manual_command(robot_command.clone(), &mut protocol, start_time, &mut last_sent_command, stats.clone());

        // Handle Server-Side Only commands
        if let Ok(mut cmd) = robot_command.try_lock() {
            if *cmd == RobotCommand::SaveMap {
                let map_name = "house_map.pgm";
                let abs_path = std::env::current_dir()
                    .map(|p| p.join(map_name))
                    .unwrap_or_else(|_| std::path::PathBuf::from(map_name));
                let abs_path_str = abs_path.to_string_lossy().to_string();

                stats.log(&format!("[SLAM] Saving house map to {}...", abs_path_str));
                if let Err(e) = slam.save_map(map_name) {
                    log::error!("Error saving map: {:?}", e);
                } else {
                    stats.log(&format!("[SLAM] Map saved successfully to {}!", abs_path_str));
                }
                *cmd = RobotCommand::StopMoving;
            } else if *cmd == RobotCommand::Reset {
                stats.log("[SERVER] Resetting map, path, and odometry...");
                odom = crate::odometry::Odometry::new();
                slam = BasicSlam::new();
                
                // Clear all state in GUI_STATE
                {
                    let mut state = crate::gui::lidar::GUI_STATE.lock().unwrap();
                    state.robot_x = 0.0;
                    state.robot_y = 0.0;
                    state.robot_theta = 0.0;
                    state.slam_x = 0.0;
                    state.slam_y = 0.0;
                    state.slam_theta = 0.0;
                    state.map_width = 0;
                    state.map_height = 0;
                    state.map_data.clear();
                    state.display_scan.clear();
                    state.frontiers.clear();
                    state.current_path.clear();
                    state.trajectory.clear();
                    state.accel_history.clear();
                    state.gyro_history.clear();
                    state.mag_history.clear();
                }

                // Force UI updates to clean dashboard displays
                let _ = gui_tx.send(GuiUpdate::Pose { x: 0.0, y: 0.0, theta: 0.0 });
                let _ = gui_tx.send(GuiUpdate::SlamPose { x: 0.0, y: 0.0, theta: 0.0 });
                let _ = gui_tx.send(GuiUpdate::Encoders { left: 0, right: 0 });
                let _ = gui_tx.send(GuiUpdate::Map { width: 0, height: 0, data: vec![] });
                let _ = gui_tx.send(GuiUpdate::Frontiers(vec![]));
                let _ = gui_tx.send(GuiUpdate::Path(vec![]));
                let _ = gui_tx.send(GuiUpdate::Lidar(vec![]));
                
                *cmd = RobotCommand::StopMoving;
                stats.log("[SERVER] Reset completed.");
            } else {
                // Determine the current navigation mode
                let current_mode = match *cmd {
                    RobotCommand::AutonomousExploration { enabled: true } => NavMode::Exploration,
                    RobotCommand::NavigateTo { x, y } => NavMode::NavigateTo(x, y),
                    _ => NavMode::None,
                };

                // Track state transitions to reset variables cleanly
                let mode_changed = match (&last_mode, &current_mode) {
                    (NavMode::None, NavMode::None) => false,
                    (NavMode::Exploration, NavMode::Exploration) => false,
                    (NavMode::NavigateTo(x1, y1), NavMode::NavigateTo(x2, y2)) => (x1 - x2).abs() > 0.001 || (y1 - y2).abs() > 0.001,
                    _ => true,
                };

                if mode_changed {
                    stats.log("[NAV] Navigation mode/goal changed. Resetting path.");
                    active_path.clear();
                    navigation_goal = None;
                    let _ = gui_tx.send(GuiUpdate::Path(vec![]));
                    let _ = gui_tx.send(GuiUpdate::NavigationTarget(None));
                    last_mode = current_mode.clone();
                    // Force immediate replan
                    last_replan_time = Instant::now() - Duration::from_secs(10);
                    last_progress_time = Instant::now();
                    last_progress_pose = current_pose;
                }

                // Event-driven check: Only run navigation planning and control if telemetry was received
                // or if the mode/goal just changed. This avoids CPU spikes and running on stale data.
                if telemetry_received || mode_changed {
                    match current_mode {
                        NavMode::None => {}
                        NavMode::Exploration | NavMode::NavigateTo(..) => {
                            // Check if we need to replan the path (throttled to 1Hz)
                            let needs_replan = active_path.is_empty() || last_replan_time.elapsed() >= Duration::from_millis(1000);
                            
                            if needs_replan {
                                // Clear old blacklisted frontiers to allow retrying them later
                                blacklisted_frontiers.retain(|(_, _, time)| time.elapsed() < Duration::from_secs(45));

                                match current_mode {
                                    NavMode::Exploration => {
                                         let frontiers = slam.get_frontiers();
                                         let _ = gui_tx.send(GuiUpdate::Frontiers(frontiers.clone()));

                                         // Log frontiers to Rerun (Yellow spheres)
                                         let frontier_points: Vec<[f32; 3]> = frontiers.iter().map(|f| [f.centroid_x, f.centroid_y, 0.1]).collect();
                                         rec.log("robot/exploration/frontiers", &rerun::Points3D::new(frontier_points)
                                             .with_colors(vec![rerun::Color::from_rgb(255, 255, 0)])
                                             .with_radii(vec![0.05])).ok();

                                         // GOAL PERSISTENCE HEURISTIC:
                                         // If we already have a target goal, we stick to it and just replan the path to it.
                                         // We only select a new best frontier target if our current target_goal is None
                                         // (e.g. it was reached, became unreachable, or timed out due to lack of progress).
                                         let mut target_goal = navigation_goal;

                                         if target_goal.is_none() {
                                             // Filter out frontiers close to blacklisted/unreachable locations,
                                             // and frontiers too close to the robot (< 0.6m) to prevent tiny local oscillations.
                                             let filtered_frontiers: Vec<_> = frontiers.iter().filter(|f| {
                                                 let dist_to_robot = ((f.centroid_x - current_pose.x).powi(2) + (f.centroid_y - current_pose.y).powi(2)).sqrt();
                                                 if dist_to_robot < 0.6 {
                                                     return false;
                                                 }
                                                 !blacklisted_frontiers.iter().any(|(bx, by, _)| {
                                                     let dx = f.centroid_x - bx;
                                                     let dy = f.centroid_y - by;
                                                     (dx*dx + dy*dy).sqrt() < 0.8
                                                 })
                                             }).collect();

                                             if let Some(&best_frontier) = filtered_frontiers.iter().max_by_key(|f| f.size) {
                                                 target_goal = Some((best_frontier.centroid_x, best_frontier.centroid_y));
                                                 stats.log(&format!("[NAV] Exploration: Selected new best frontier target: X={:.2}, Y={:.2}", best_frontier.centroid_x, best_frontier.centroid_y));
                                             }
                                         }

                                         if let Some((gx, gy)) = target_goal {
                                             stats.log(&format!("[NAV] Exploration: Planning path to target: X={:.2}, Y={:.2}", gx, gy));
                                             if let Some(path) = slam.plan_path(gx, gy) {
                                                 active_path = path;
                                                 navigation_goal = Some((gx, gy));
                                                 let _ = gui_tx.send(GuiUpdate::Path(active_path.clone()));
                                                 let _ = gui_tx.send(GuiUpdate::NavigationTarget(navigation_goal));
                                                 
                                                 // Log path to Rerun (Blue line)
                                                 let path_points: Vec<[f32; 3]> = active_path.iter().map(|p| [p.0, p.1, 0.05]).collect();
                                                 rec.log("robot/exploration/path", &rerun::LineStrips3D::new([path_points])
                                                     .with_colors(vec![rerun::Color::from_rgb(0, 128, 255)])).ok();
                                                 
                                                 last_progress_time = Instant::now();
                                                 last_progress_pose = current_pose;
                                             } else {
                                                 stats.log(&format!("[NAV] Exploration: A* failed to find path to X={:.2}, Y={:.2}. Blacklisting.", gx, gy));
                                                 blacklisted_frontiers.push((gx, gy, Instant::now()));
                                                 active_path.clear();
                                                 navigation_goal = None;
                                                 let _ = gui_tx.send(GuiUpdate::Path(vec![]));
                                                 let _ = gui_tx.send(GuiUpdate::NavigationTarget(None));
                                             }
                                         } else {
                                             stats.log("[NAV] Exploration: No reachable frontiers found.");
                                             active_path.clear();
                                             navigation_goal = None;
                                             let _ = gui_tx.send(GuiUpdate::Path(vec![]));
                                             let _ = gui_tx.send(GuiUpdate::NavigationTarget(None));
                                         }
                                    }
                                    NavMode::NavigateTo(gx, gy) => {
                                        stats.log(&format!("[NAV] Go To: Planning path to goal: X={:.2}, Y={:.2}", gx, gy));
                                        if let Some(path) = slam.plan_path(gx, gy) {
                                            active_path = path;
                                            navigation_goal = Some((gx, gy));
                                            let _ = gui_tx.send(GuiUpdate::Path(active_path.clone()));
                                            let _ = gui_tx.send(GuiUpdate::NavigationTarget(navigation_goal));
                                            
                                            // Log path to Rerun (Blue line)
                                            let path_points: Vec<[f32; 3]> = active_path.iter().map(|p| [p.0, p.1, 0.05]).collect();
                                            rec.log("robot/exploration/path", &rerun::LineStrips3D::new([path_points])
                                                .with_colors(vec![rerun::Color::from_rgb(0, 128, 255)])).ok();
                                            
                                            last_progress_time = Instant::now();
                                            last_progress_pose = current_pose;
                                        } else {
                                            stats.log("[NAV] Go To: A* failed to find path to goal! Target is unreachable.");
                                            *cmd = RobotCommand::StopMoving;
                                            active_path.clear();
                                            navigation_goal = None;
                                            let _ = gui_tx.send(GuiUpdate::Path(vec![]));
                                            let _ = gui_tx.send(GuiUpdate::NavigationTarget(None));

                                            // Send immediate STOP command
                                            let temp_cmd = RobotCommand::StopMoving;
                                            if temp_cmd != last_sent_command {
                                                let millis = start_time.elapsed().as_millis() as u32;
                                                let msg = crate::homerobot::ServerToRobotMessage {
                                                    sequence_millis: millis,
                                                    payload: temp_cmd.into_payload(),
                                                };
                                                let mut buf = Vec::new();
                                                prost::Message::encode(&msg, &mut buf).unwrap();
                                                let mut final_packet = (buf.len() as u16).to_be_bytes().to_vec();
                                                final_packet.extend(buf);
                                                let _ = protocol.send_packet(&final_packet);
                                                last_sent_command = temp_cmd;
                                            }
                                        }
                                    }
                                    _ => unreachable!()
                                }
                                last_replan_time = Instant::now();
                            }

                            // Stuck Detection: Check if robot has made progress
                            if !active_path.is_empty() {
                                if let Some(goal) = navigation_goal {
                                    let dist_moved = ((current_pose.x - last_progress_pose.x).powi(2) + (current_pose.y - last_progress_pose.y).powi(2)).sqrt();
                                    let angle_moved = (current_pose.theta - last_progress_pose.theta).abs();
                                    
                                    // If we've translated more than 3cm or rotated more than 5 degrees, we have made progress
                                    if dist_moved > 0.03 || angle_moved > 0.08 {
                                        last_progress_time = Instant::now();
                                        last_progress_pose = current_pose;
                                    }

                                    // If we haven't made progress for 8 seconds, cancel/blacklist
                                    if last_progress_time.elapsed() >= Duration::from_secs(8) {
                                        match current_mode {
                                            NavMode::Exploration => {
                                                stats.log(&format!("[NAV] Stuck! Exploration progress timeout. Blacklisting frontier at X={:.2}, Y={:.2}", goal.0, goal.1));
                                                blacklisted_frontiers.push((goal.0, goal.1, Instant::now()));
                                                active_path.clear();
                                                navigation_goal = None;
                                                let _ = gui_tx.send(GuiUpdate::Path(vec![]));
                                                let _ = gui_tx.send(GuiUpdate::NavigationTarget(None));
                                            }
                                            NavMode::NavigateTo(..) => {
                                                stats.log(&format!("[NAV] Stuck! Target is unreachable. Cancelling navigation to X={:.2}, Y={:.2}", goal.0, goal.1));
                                                *cmd = RobotCommand::StopMoving;
                                                active_path.clear();
                                                navigation_goal = None;
                                                let _ = gui_tx.send(GuiUpdate::Path(vec![]));
                                                let _ = gui_tx.send(GuiUpdate::NavigationTarget(None));

                                                // Send immediate STOP command
                                                let millis = start_time.elapsed().as_millis() as u32;
                                                let msg = crate::homerobot::ServerToRobotMessage {
                                                    sequence_millis: millis,
                                                    payload: RobotCommand::StopMoving.into_payload(),
                                                };
                                                let mut buf = Vec::new();
                                                prost::Message::encode(&msg, &mut buf).unwrap();
                                                let mut final_packet = (buf.len() as u16).to_be_bytes().to_vec();
                                                final_packet.extend(buf);
                                                let _ = protocol.send_packet(&final_packet);
                                            }
                                            _ => {}
                                        }
                                    }
                                }
                            }

                            // Check goal completion
                            if !active_path.is_empty() {
                                if let Some(goal) = navigation_goal {
                                    let dx = goal.0 - current_pose.x;
                                    let dy = goal.1 - current_pose.y;
                                    let dist = (dx*dx + dy*dy).sqrt();
                                    if dist < 0.20 { // 20cm tolerance
                                        stats.log(&format!("[NAV] Goal reached within {:.2}m!", dist));
                                        *cmd = RobotCommand::StopMoving;
                                        let _ = gui_tx.send(GuiUpdate::NavigationTarget(None));
                                        let _ = gui_tx.send(GuiUpdate::Path(vec![]));
                                        active_path.clear();
                                        navigation_goal = None;
                                        
                                        // Send immediate STOP command
                                        let millis = start_time.elapsed().as_millis() as u32;
                                        let msg = crate::homerobot::ServerToRobotMessage {
                                            sequence_millis: millis,
                                            payload: RobotCommand::StopMoving.into_payload(),
                                        };
                                        let mut buf = Vec::new();
                                        prost::Message::encode(&msg, &mut buf).unwrap();
                                        let mut final_packet = (buf.len() as u16).to_be_bytes().to_vec();
                                        final_packet.extend(buf);
                                        let _ = protocol.send_packet(&final_packet);
                                    }
                                }
                            }

                            // Follow active path if it is still valid
                            if !active_path.is_empty() {
                                let lookahead_distance = 0.25_f32;
                                let mut target_waypoint = None;
                                for &waypoint in &active_path {
                                    let dx = waypoint.0 - current_pose.x;
                                    let dy = waypoint.1 - current_pose.y;
                                    let dist = (dx*dx + dy*dy).sqrt();
                                    if dist > lookahead_distance {
                                        target_waypoint = Some(waypoint);
                                        break;
                                    }
                                }
                                let target_waypoint = target_waypoint.or_else(|| active_path.last().cloned()).unwrap();

                                let dx = target_waypoint.0 - current_pose.x;
                                let dy = target_waypoint.1 - current_pose.y;
                                let target_theta = dy.atan2(dx);

                                let mut angle_diff = target_theta - current_pose.theta;
                                while angle_diff > std::f32::consts::PI { angle_diff -= 2.0 * std::f32::consts::PI; }
                                while angle_diff < -std::f32::consts::PI { angle_diff += 2.0 * std::f32::consts::PI; }

                                let (l_pwr, l_dir, r_pwr, r_dir) = if min_front_dist < 0.35 {
                                    // Obstacle! Rotate in place
                                    if angle_diff > 0.0 {
                                        (120, -1.0, 120, 1.0)
                                    } else {
                                        (120, 1.0, 120, -1.0)
                                    }
                                } else if angle_diff.abs() > 0.4 {
                                    // Rotate in place to align with path waypoint
                                    if angle_diff > 0.0 {
                                        (120, -1.0, 120, 1.0)
                                    } else {
                                        (120, 1.0, 120, -1.0)
                                    }
                                } else {
                                    // Blend forward and turn
                                    let base_pwr = 100;
                                    let turn = (angle_diff * 50.0) as i32;
                                    let left = (base_pwr - turn).clamp(40, 150) as u8;
                                    let right = (base_pwr + turn).clamp(40, 150) as u8;
                                    (left, 1.0, right, 1.0)
                                };

                                let temp_cmd = RobotCommand::MotorAngle {
                                    left_power: l_pwr,
                                    left_angle: l_dir,
                                    right_power: r_pwr,
                                    right_angle: r_dir,
                                };
                                if temp_cmd != last_sent_command {
                                    let millis = start_time.elapsed().as_millis() as u32;
                                    let msg = crate::homerobot::ServerToRobotMessage {
                                        sequence_millis: millis,
                                        payload: temp_cmd.into_payload(),
                                    };
                                    let mut buf = Vec::new();
                                    prost::Message::encode(&msg, &mut buf).unwrap();
                                    let mut final_packet = (buf.len() as u16).to_be_bytes().to_vec();
                                    final_packet.extend(buf);
                                    let _ = protocol.send_packet(&final_packet);
                                    last_sent_command = temp_cmd;
                                }
                            }
                        }
                    }
                }
            }
        }

        sleep(Duration::from_millis(10));
    }

    stats.active_connections.fetch_sub(1, Ordering::SeqCst);
    stats.log(&format!("[CONN] Closing connection to {}", addr));
    let _ = gui_tx.send(GuiUpdate::Status("Idle".to_string()));
}
