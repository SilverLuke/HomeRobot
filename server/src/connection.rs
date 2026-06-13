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
    
    while stats.running.load(Ordering::Relaxed) && sig_count.load(Ordering::Relaxed) == 0 {
        let elapsed = start_time.elapsed().as_secs_f64();
        let rec = rec.lock().unwrap().clone();
        rec.set_time("realtime", std::time::Duration::from_secs_f64(elapsed));

        loop {
            match protocol.read_message() {
                Ok(Some(msg)) => {
                    if let Some(payload) = msg.payload {
                        match payload {
                            Payload::Battery(bat) => {
                                println!("[SERVER] Telemetry Heartbeat: Battery {}%", bat.percentage);
                                let _ = gui_tx.send(GuiUpdate::Battery { percentage: bat.percentage, voltage_mv: bat.voltage_mv });
                                rec.log("robot/battery", &rerun::Scalars::new([bat.percentage as f64])).ok();
                            }
                            Payload::Encoders(enc) => {
                                println!("[SERVER] Telemetry Heartbeat: Encoders L={} R={}", enc.left_encoder, enc.right_encoder);
                                odom.update(enc.left_encoder, enc.right_encoder);
                                let _ = gui_tx.send(GuiUpdate::Encoders { left: enc.left_encoder, right: enc.right_encoder });
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
                                if let (Some(a), Some(g)) = (imu.acceleration, imu.gyroscope) {
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
                                min_front_dist = 10.0;
                                for p in &scan.points {
                                    if p.angle_deg < 40.0 || p.angle_deg > 320.0 {
                                        let d = p.distance_mm / 1000.0;
                                        if d > 0.05 && d < min_front_dist {
                                            min_front_dist = d;
                                        }
                                    }
                                }
                                let corrected_pose = slam.update(&scan.points, &odom.pose);
                                let _ = gui_tx.send(GuiUpdate::Lidar(scan.points.clone()));
                                let _ = gui_tx.send(GuiUpdate::SlamPose { 
                                    x: corrected_pose.x, 
                                    y: corrected_pose.y, 
                                    theta: corrected_pose.theta 
                                });

                                // Log SLAM Pose to Rerun
                                rec.log("robot/pose/slam", &rerun::Transform3D::from_translation_rotation(
                                    [corrected_pose.x, corrected_pose.y, 0.0],
                                    rerun::RotationAxisAngle::new([0.0, 0.0, 1.0], rerun::Angle::from_radians(corrected_pose.theta))
                                )).ok();

                                // Log Lidar Points to Rerun (in Robot Frame)
                                let points: Vec<[f32; 3]> = scan.points.iter().map(|p| {
                                    let angle = (p.angle_deg as f32).to_radians();
                                    let d = p.distance_mm / 1000.0;
                                    [d * angle.cos(), d * angle.sin(), 0.0]
                                }).collect();
                                rec.log("robot/sensor/lidar", &rerun::Points3D::new(points).with_colors(vec![rerun::Color::from_rgb(0, 255, 0)])).ok();

                                // Send map update every scan (optimized later)
                                let (w, h, data) = slam.get_map_data();
                                let _ = gui_tx.send(GuiUpdate::Map { 
                                    width: w, 
                                    height: h, 
                                    data: data.to_vec() 
                                });
                            }
                            Payload::Config(conf) => {
                                stats.log("[CONFIG] Robot configuration received");
                                let _ = gui_tx.send(GuiUpdate::Config(conf));
                            }
                            Payload::RpcResponse(resp) => {
                                stats.log(&format!("[RPC RESPONSE] ID: {}, Error: {}", resp.call_id, resp.error));
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
                stats.log("[SLAM] Saving house map to house_map.pgm...");
                if let Err(e) = slam.save_map("house_map.pgm") {
                    eprintln!("Error saving map: {:?}\r", e);
                } else {
                    stats.log("[SLAM] Map saved successfully!");
                }
                *cmd = RobotCommand::StopMoving;
            } else if let RobotCommand::AutonomousExploration { enabled } = *cmd {
                if enabled {
                    let frontiers = slam.get_frontiers();
                    let _ = gui_tx.send(GuiUpdate::Frontiers(frontiers.clone()));

                    // Log frontiers to Rerun (Yellow spheres)
                    let frontier_points: Vec<[f32; 3]> = frontiers.iter().map(|f| [f.centroid_x, f.centroid_y, 0.1]).collect();
                    rec.log("robot/exploration/frontiers", &rerun::Points3D::new(frontier_points)
                        .with_colors(vec![rerun::Color::from_rgb(255, 255, 0)])
                        .with_radii(vec![0.05])).ok();

                    if let Some(best_frontier) = frontiers.iter().max_by_key(|f| f.size) {
                        if let Some(path) = slam.plan_path(best_frontier.centroid_x, best_frontier.centroid_y) {
                            let _ = gui_tx.send(GuiUpdate::Path(path.clone()));

                            // Log path to Rerun (Blue line)
                            let path_points: Vec<[f32; 3]> = path.iter().map(|p| [p.0, p.1, 0.05]).collect();
                            rec.log("robot/exploration/path", &rerun::LineStrips3D::new([path_points])
                                .with_colors(vec![rerun::Color::from_rgb(0, 128, 255)])).ok();
                            
                            if let Some(next_waypoint) = path.get(2).or_else(|| path.get(0)) {
                                // Simple Controller
                                let dx = next_waypoint.0 - odom.pose.x;
                                let dy = next_waypoint.1 - odom.pose.y;
                                let target_theta = dy.atan2(dx);
                                let _dist = (dx*dx + dy*dy).sqrt();

                                let mut angle_diff = target_theta - odom.pose.theta;
                                while angle_diff > std::f32::consts::PI { angle_diff -= 2.0 * std::f32::consts::PI; }
                                while angle_diff < -std::f32::consts::PI { angle_diff += 2.0 * std::f32::consts::PI; }

                                let (l_pwr, r_pwr) = if min_front_dist < 0.35 {
                                    // Obstacle too close! Rotate right to avoid
                                    (100, 0)
                                } else if angle_diff.abs() > 0.5 {
                                    // Rotate in place
                                    let pwr = 80;
                                    if angle_diff > 0.0 { (0, pwr) } else { (pwr, 0) }
                                } else {
                                    // Drive forward with slight turn
                                    let pwr = 100;
                                    let turn = (angle_diff * 40.0) as i32;
                                    ((pwr - turn).clamp(0, 255) as u8, (pwr + turn).clamp(0, 255) as u8)
                                };

                                // Send the command directly without overwriting the Exploration State!
                                let temp_cmd = RobotCommand::MotorAngle {
                                    left_power: l_pwr,
                                    left_angle: 1.0,
                                    right_power: r_pwr,
                                    right_angle: 1.0,
                                };
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
