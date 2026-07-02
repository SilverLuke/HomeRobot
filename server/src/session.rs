//! Per-connection robot session, structured as an explicit event loop.
//!
//! Three producer threads feed one `SessionEvent` queue:
//!   - a reader thread decoding robot messages from the socket,
//!   - a pump forwarding commands from the [`CommandBus`] (GUI / proxy),
//!   - the loop itself synthesizes `Tick` on receive timeout so stuck
//!     detection runs even when telemetry stalls.
//!
//! The session consumes events single-threaded: no shared mutable state, no
//! polling of command cells. All robot-bound traffic goes through
//! [`RobotLink`], the single place that knows the wire framing.

use std::collections::HashMap;
use std::io;
use std::io::Write;
use std::net::TcpStream;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use prost::Message;

use crate::command::{Command, CommandBus, RpcResult};
use crate::command::NavMode;
use crate::gui::GuiUpdate;
use crate::homerobot::robot_to_server_message::Payload;
use crate::homerobot::server_to_robot_message::Payload as OutPayload;
use crate::homerobot::{RobotToServerMessage, ServerToRobotMessage};
use crate::navigator::{NavUpdate, Navigator};
use crate::odometry::{Odometry, Pose};
use crate::reader::ProtocolManager;
use crate::scan::{CompletedSweep, ScanAssembler};
use crate::slam::{BasicSlam, Slam};
use crate::stats::Stats;

/// Lidar is switched on as soon as a robot connects.
const DEFAULT_LIDAR_HZ: f32 = 5.0;
/// Receive timeout of the event loop; doubles as the navigation tick period
/// when no telemetry is arriving.
const EVENT_POLL_INTERVAL: Duration = Duration::from_millis(100);
const READ_TIMEOUT: Duration = Duration::from_millis(500);

pub enum SessionEvent {
    FromRobot(RobotToServerMessage),
    Command(Command),
    /// Periodic heartbeat, synthesized when no event arrived for a while.
    Tick,
    /// The reader thread lost the connection.
    Disconnected(String),
}

/// Write half of the robot connection: message framing, sequence numbers and
/// suppression of duplicate consecutive drive commands (the navigator re-emits
/// its drive decision on every telemetry event).
///
/// Generic over the sink so tests can drive a session against a `Vec<u8>`.
struct RobotLink<W: Write> {
    stream: W,
    start_time: Instant,
    stats: Arc<Stats>,
    last_drive: Option<(u8, f32, u8, f32)>,
}

impl<W: Write> RobotLink<W> {
    fn send(&mut self, payload: OutPayload) -> io::Result<()> {
        let msg = ServerToRobotMessage {
            sequence_millis: self.start_time.elapsed().as_millis() as u32,
            payload: Some(payload),
        };
        let mut buf = Vec::new();
        msg.encode(&mut buf).expect("encoding into Vec cannot fail");
        let mut packet = (buf.len() as u16).to_be_bytes().to_vec();
        packet.extend(buf);
        self.stream.write_all(&packet)?;
        self.stream.flush()?;
        self.stats.total_tx.fetch_add(packet.len(), Ordering::SeqCst);
        Ok(())
    }

    fn send_drive(&mut self, left_power: u8, left_angle: f32, right_power: u8, right_angle: f32) -> io::Result<()> {
        let key = (left_power, left_angle, right_power, right_angle);
        if self.last_drive == Some(key) {
            return Ok(());
        }
        self.send(OutPayload::MotorMove(crate::homerobot::MotorMoveCommand {
            left_power: left_power as u32,
            left_angle,
            right_power: right_power as u32,
            right_angle,
        }))?;
        self.last_drive = Some(key);
        Ok(())
    }

    fn stop(&mut self) -> io::Result<()> {
        self.send_drive(0, 0.0, 0, 0.0)
    }
}

struct PendingRpc {
    method: String,
    reply: Option<mpsc::Sender<RpcResult>>,
}

pub struct Session<W: Write> {
    link: RobotLink<W>,
    odom: Odometry,
    slam: BasicSlam,
    navigator: Navigator,
    scan: ScanAssembler,
    /// Best pose estimate: SLAM-corrected on each sweep, dead-reckoned from
    /// encoder deltas in between.
    pose: Pose,
    pose_initialized: bool,
    min_front_dist: f32,
    pending_rpcs: HashMap<u32, PendingRpc>,
    next_call_id: u32,
    gui_tx: mpsc::Sender<GuiUpdate>,
    stats: Arc<Stats>,
    rec: Arc<Mutex<rerun::RecordingStream>>,
    start_time: Instant,
}

/// Entry point for a robot connection; returns when the connection drops or
/// the server shuts down.
pub fn run_session(
    stream: TcpStream,
    bus: CommandBus,
    stats: Arc<Stats>,
    sig_count: Arc<AtomicUsize>,
    gui_tx: mpsc::Sender<GuiUpdate>,
    rec: Arc<Mutex<rerun::RecordingStream>>,
) {
    stats.active_connections.fetch_add(1, Ordering::SeqCst);
    let addr = stream
        .peer_addr()
        .map(|a| a.to_string())
        .unwrap_or_else(|_| "unknown".to_string());
    stats.log(&format!("[CONN] New connection from {}", addr));
    let _ = gui_tx.send(GuiUpdate::Status(format!("Connected: {}", addr)));

    let (event_tx, event_rx) = mpsc::channel::<SessionEvent>();

    // Reader thread: owns the read half, produces FromRobot / Disconnected.
    let read_stream = match stream.try_clone() {
        Ok(s) => s,
        Err(e) => {
            log::error!("[CONN] Could not clone stream for {}: {}", addr, e);
            stats.active_connections.fetch_sub(1, Ordering::SeqCst);
            return;
        }
    };
    read_stream.set_read_timeout(Some(READ_TIMEOUT)).ok();
    {
        let stats = stats.clone();
        let sig_count = sig_count.clone();
        let tx = event_tx.clone();
        thread::spawn(move || reader_thread(read_stream, stats, sig_count, tx));
    }

    // Command pump: forwards bus commands into the event queue. Exits when
    // the session drops the receiver; the bus prunes its sender on next send.
    {
        let cmd_rx = bus.subscribe();
        let tx = event_tx;
        thread::spawn(move || {
            for cmd in cmd_rx {
                if tx.send(SessionEvent::Command(cmd)).is_err() {
                    return;
                }
            }
        });
    }

    let mut session = Session::new(stream, stats.clone(), gui_tx.clone(), rec);
    if let Err(e) = session.link.send(OutPayload::LidarControl(crate::homerobot::LidarControlCommand {
        active: true,
        target_frequency_hz: DEFAULT_LIDAR_HZ,
    })) {
        log::error!("[CONN] Failed to enable lidar on connect: {}", e);
    }

    while stats.running.load(Ordering::Relaxed) && sig_count.load(Ordering::Relaxed) == 0 {
        match event_rx.recv_timeout(EVENT_POLL_INTERVAL) {
            Ok(SessionEvent::Disconnected(reason)) => {
                stats.log(&format!("[ERROR] Connection to {} lost: {}", addr, reason));
                let _ = gui_tx.send(GuiUpdate::Status("Disconnected".to_string()));
                break;
            }
            Ok(event) => session.handle_event(event),
            Err(mpsc::RecvTimeoutError::Timeout) => session.handle_event(SessionEvent::Tick),
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        }
    }

    session.fail_pending_rpcs("connection closed");
    stats.active_connections.fetch_sub(1, Ordering::SeqCst);
    stats.log(&format!("[CONN] Closing connection to {}", addr));
    let _ = gui_tx.send(GuiUpdate::Status("Idle".to_string()));
}

fn reader_thread(
    stream: TcpStream,
    stats: Arc<Stats>,
    sig_count: Arc<AtomicUsize>,
    tx: mpsc::Sender<SessionEvent>,
) {
    let mut protocol = ProtocolManager::new(stream, stats.clone());
    while stats.running.load(Ordering::Relaxed) && sig_count.load(Ordering::Relaxed) == 0 {
        match protocol.read_message() {
            Ok(Some(msg)) => {
                if tx.send(SessionEvent::FromRobot(msg)).is_err() {
                    return;
                }
            }
            Ok(None) => continue, // read timeout or partial message
            Err(e) => {
                let _ = tx.send(SessionEvent::Disconnected(format!("{} ({:?})", e, e.kind())));
                return;
            }
        }
    }
}

impl<W: Write> Session<W> {
    fn new(
        stream: W,
        stats: Arc<Stats>,
        gui_tx: mpsc::Sender<GuiUpdate>,
        rec: Arc<Mutex<rerun::RecordingStream>>,
    ) -> Self {
        let start_time = Instant::now();
        Self {
            link: RobotLink {
                stream,
                start_time,
                stats: stats.clone(),
                last_drive: None,
            },
            odom: Odometry::new(),
            slam: BasicSlam::new(),
            navigator: Navigator::new(),
            scan: ScanAssembler::new(),
            pose: Pose { x: 0.0, y: 0.0, theta: 0.0 },
            pose_initialized: false,
            min_front_dist: 10.0,
            pending_rpcs: HashMap::new(),
            next_call_id: 1,
            gui_tx,
            stats,
            rec,
            start_time,
        }
    }

    /// Rerun stream with the session timeline set to "now".
    fn rec(&self) -> rerun::RecordingStream {
        let rec = self.rec.lock().unwrap().clone();
        rec.set_time("realtime", self.start_time.elapsed());
        rec
    }

    pub fn handle_event(&mut self, event: SessionEvent) {
        match event {
            SessionEvent::FromRobot(msg) => {
                let seq = msg.sequence_millis;
                if let Some(payload) = msg.payload {
                    self.handle_robot_payload(payload, seq);
                }
            }
            SessionEvent::Command(cmd) => self.handle_command(cmd),
            SessionEvent::Tick => self.run_navigation(),
            SessionEvent::Disconnected(_) => unreachable!("handled by the session loop"),
        }
    }

    // ---- Robot telemetry -------------------------------------------------

    fn handle_robot_payload(&mut self, payload: Payload, sequence_millis: u32) {
        match payload {
            Payload::Battery(bat) => {
                log::info!("[SERVER] Telemetry Heartbeat: Battery {}%", bat.percentage);
                let _ = self.gui_tx.send(GuiUpdate::Battery {
                    percentage: bat.percentage,
                    voltage_mv: bat.voltage_mv,
                });
                self.rec().log("robot/battery", &rerun::Scalars::new([bat.percentage as f64])).ok();
            }
            Payload::Encoders(enc) => {
                self.handle_encoders(enc.left_encoder, enc.right_encoder, sequence_millis);
                self.run_navigation();
            }
            Payload::Imu(imu) => {
                if let (Some(a), Some(g)) = (imu.acceleration, imu.gyroscope) {
                    self.odom.update_imu(g.z, sequence_millis);
                    let (mx, my, mz) = imu.magnetometer.map(|m| (m.x, m.y, m.z)).unwrap_or((0.0, 0.0, 0.0));
                    let _ = self.gui_tx.send(GuiUpdate::Imu {
                        ax: a.x, ay: a.y, az: a.z,
                        gx: g.x, gy: g.y, gz: g.z,
                        mx, my, mz,
                    });
                    let rec = self.rec();
                    rec.log("robot/imu/accel", &rerun::Arrows3D::from_vectors([[a.x, a.y, a.z]])).ok();
                    rec.log("robot/imu/gyro/z", &rerun::Scalars::new([g.z as f64])).ok();
                }
                self.run_navigation();
            }
            Payload::Lidar(scan) => {
                for sweep in self.scan.push(scan.points) {
                    self.process_sweep(sweep);
                }
                self.run_navigation();
            }
            Payload::Config(conf) => {
                self.stats.log("[CONFIG] Robot configuration received");
                let _ = self.gui_tx.send(GuiUpdate::Config(conf));
            }
            Payload::Capabilities(cap) => {
                self.stats.log(&format!(
                    "[CAPABILITIES] Accel: {}, Gyro: {}, Mag: {}, Wheel Dia: {}mm, Track: {}mm, Ticks/Rev: {}",
                    cap.has_accelerometer, cap.has_gyroscope, cap.has_magnetometer,
                    cap.wheel_diameter_mm, cap.wheel_track_mm, cap.encoder_ticks_per_rev
                ));
                self.odom.update_sizes(cap.wheel_diameter_mm, cap.wheel_track_mm, cap.encoder_ticks_per_rev);
                {
                    let mut sizes = crate::constants::ROBOT_SIZES.lock().unwrap();
                    sizes.wheel_base = cap.wheel_track_mm / 1000.0;
                    sizes.ticks_per_meter =
                        (cap.encoder_ticks_per_rev as f32) / (std::f32::consts::PI * (cap.wheel_diameter_mm / 1000.0));
                }
                let _ = self.gui_tx.send(GuiUpdate::Capabilities {
                    _has_accelerometer: cap.has_accelerometer,
                    _has_gyroscope: cap.has_gyroscope,
                    has_magnetometer: cap.has_magnetometer,
                    _wheel_diameter_mm: cap.wheel_diameter_mm,
                    _wheel_track_mm: cap.wheel_track_mm,
                    _encoder_ticks_per_rev: cap.encoder_ticks_per_rev,
                });
            }
            Payload::RpcResponse(resp) => self.handle_rpc_response(resp),
            _ => {}
        }
    }

    fn handle_encoders(&mut self, left: i32, right: i32, sequence_millis: u32) {
        if left != 0 || right != 0 {
            log::info!("[SERVER] Telemetry Heartbeat: Encoders delta L={} R={}", left, right);
        }
        let old_odom_pose = self.odom.pose;
        self.odom.update_encoders(left, right, sequence_millis);
        self.slam.add_odom_pose(self.odom.pose, Instant::now());

        if !self.pose_initialized {
            self.pose = self.odom.pose;
            self.pose_initialized = true;
        } else {
            // Propagate the odometry delta into the SLAM-corrected pose so the
            // path follower doesn't act on stale position between sweeps.
            let dx = self.odom.pose.x - old_odom_pose.x;
            let dy = self.odom.pose.y - old_odom_pose.y;
            let dt = self.odom.pose.theta - old_odom_pose.theta;

            // Rotate the translation delta into the corrected pose's frame.
            let diff_theta = self.pose.theta - old_odom_pose.theta;
            let (sin_diff, cos_diff) = diff_theta.sin_cos();
            self.pose.x += dx * cos_diff - dy * sin_diff;
            self.pose.y += dx * sin_diff + dy * cos_diff;
            self.pose.theta += dt;
            while self.pose.theta > std::f32::consts::PI {
                self.pose.theta -= 2.0 * std::f32::consts::PI;
            }
            while self.pose.theta < -std::f32::consts::PI {
                self.pose.theta += 2.0 * std::f32::consts::PI;
            }

            let _ = self.gui_tx.send(GuiUpdate::SlamPose {
                x: self.pose.x,
                y: self.pose.y,
                theta: self.pose.theta,
            });
            self.log_slam_pose();
        }

        let _ = self.gui_tx.send(GuiUpdate::Encoders {
            left: self.odom.cumulative_left,
            right: self.odom.cumulative_right,
        });
        let _ = self.gui_tx.send(GuiUpdate::Pose {
            x: self.odom.pose.x,
            y: self.odom.pose.y,
            theta: self.odom.pose.theta,
        });
        self.rec().log(
            "robot/pose/odom",
            &rerun::Transform3D::from_translation_rotation(
                [self.odom.pose.x, self.odom.pose.y, 0.0],
                rerun::RotationAxisAngle::new([0.0, 0.0, 1.0], rerun::Angle::from_radians(self.odom.pose.theta)),
            ),
        ).ok();
    }

    fn process_sweep(&mut self, sweep: CompletedSweep) {
        if let Some(summary) = sweep.summary {
            self.stats.log(&summary);
        }
        if let Some(hz) = sweep.rate_hz {
            let _ = self.gui_tx.send(GuiUpdate::LidarScanRate(hz));
        }

        // Obstacle proximity in the frontal cone.
        self.min_front_dist = 10.0;
        for pt in &sweep.points {
            if pt.angle_deg < 40.0 || pt.angle_deg > 320.0 {
                let d = pt.distance_mm / 1000.0;
                if d > 0.05 && d < self.min_front_dist {
                    self.min_front_dist = d;
                }
            }
        }

        // SLAM correction on the complete, consistent sweep.
        self.pose = self.slam.update(&sweep.points, &self.odom.pose);
        self.pose_initialized = true;
        let _ = self.gui_tx.send(GuiUpdate::Lidar(sweep.points.clone()));
        let _ = self.gui_tx.send(GuiUpdate::SlamPose {
            x: self.pose.x,
            y: self.pose.y,
            theta: self.pose.theta,
        });

        let rec = self.rec();
        self.log_slam_pose();
        let (rerun_pts, rerun_colors): (Vec<[f32; 3]>, Vec<rerun::Color>) = sweep.points.iter().map(|pt| {
            let angle = -pt.angle_deg.to_radians();
            let d = pt.distance_mm / 1000.0;
            let t = (pt.quality as f32 / 15.0).clamp(0.0, 1.0);
            let r = ((1.0 - t) * 255.0) as u8;
            let g = (t * 255.0) as u8;
            ([d * angle.cos(), d * angle.sin(), 0.0], rerun::Color::from_rgb(r, g, 0))
        }).unzip();
        rec.log("robot/sensor/lidar", &rerun::Points3D::new(rerun_pts).with_colors(rerun_colors)).ok();

        let (w, h, data) = self.slam.get_map_data();
        let _ = self.gui_tx.send(GuiUpdate::Map {
            width: w,
            height: h,
            data: data.to_vec(),
        });
    }

    fn log_slam_pose(&self) {
        self.rec().log(
            "robot/pose/slam",
            &rerun::Transform3D::from_translation_rotation(
                [self.pose.x, self.pose.y, 0.0],
                rerun::RotationAxisAngle::new([0.0, 0.0, 1.0], rerun::Angle::from_radians(self.pose.theta)),
            ),
        ).ok();
    }

    // ---- Commands --------------------------------------------------------

    fn handle_command(&mut self, cmd: Command) {
        match cmd {
            Command::Drive { left_power, left_angle, right_power, right_angle } => {
                self.enter_manual();
                if let Err(e) = self.link.send_drive(left_power, left_angle, right_power, right_angle) {
                    log::error!("[CMD] Failed to send drive command: {}", e);
                }
            }
            Command::StopMoving => {
                self.enter_manual();
                if let Err(e) = self.link.stop() {
                    log::error!("[CMD] Failed to send stop: {}", e);
                }
            }
            Command::StopAll => {
                self.enter_manual();
                if let Err(e) = self.link.send(OutPayload::StopAll(true)) {
                    log::error!("[CMD] Failed to send StopAll: {}", e);
                }
            }
            Command::LidarControl { active, target_frequency_hz } => {
                if let Err(e) = self.link.send(OutPayload::LidarControl(crate::homerobot::LidarControlCommand {
                    active,
                    target_frequency_hz,
                })) {
                    log::error!("[CMD] Failed to send lidar control: {}", e);
                }
            }
            Command::UpdateConfig {
                left_kp, left_ki, left_kd,
                right_kp, right_ki, right_kd,
                lidar_frequency,
            } => {
                let config = crate::homerobot::RobotConfig {
                    left_motor: Some(crate::homerobot::MotorPidConfig {
                        kp: left_kp, ki: left_ki, kd: left_kd, max_speed: 255,
                    }),
                    right_motor: Some(crate::homerobot::MotorPidConfig {
                        kp: right_kp, ki: right_ki, kd: right_kd, max_speed: 255,
                    }),
                    lidar_frequency,
                };
                if let Err(e) = self.link.send(OutPayload::MotorConfig(config)) {
                    log::error!("[CMD] Failed to send config: {}", e);
                }
            }
            Command::RunDiagnostic => self.send_rpc("RunDiagnostic", Vec::new(), None),
            Command::ExecuteMotion { motion_type, left_ticks, right_ticks, max_power, reply } => {
                self.enter_manual();
                let req = crate::homerobot::MotionRequest {
                    r#type: motion_type,
                    distance: 0.0,
                    angle: 0.0,
                    radius: 0.0,
                    max_power,
                    left_ticks,
                    right_ticks,
                };
                let mut payload = Vec::new();
                req.encode(&mut payload).expect("encoding into Vec cannot fail");
                self.send_rpc("ExecuteMotion", payload, reply);
            }
            Command::SaveMap => {
                let map_name = "house_map.pgm";
                let abs_path = std::env::current_dir()
                    .map(|p| p.join(map_name))
                    .unwrap_or_else(|_| std::path::PathBuf::from(map_name));
                self.stats.log(&format!("[SLAM] Saving house map to {}...", abs_path.display()));
                match self.slam.save_map(map_name) {
                    Ok(()) => self.stats.log(&format!("[SLAM] Map saved successfully to {}!", abs_path.display())),
                    Err(e) => log::error!("Error saving map: {:?}", e),
                }
            }
            Command::Reset => self.reset(),
            Command::SetMode(mode) => {
                let updates = self.navigator.set_mode(mode, self.pose);
                self.apply_nav_updates(updates);
                // Plan immediately instead of waiting for the next telemetry event.
                self.run_navigation();
            }
        }
    }

    /// Any manual command cancels autonomous navigation.
    fn enter_manual(&mut self) {
        let updates = self.navigator.set_mode(NavMode::Manual, self.pose);
        self.apply_nav_updates(updates);
    }

    fn reset(&mut self) {
        self.stats.log("[SERVER] Resetting map, path, and odometry...");
        let _ = self.link.stop();
        self.odom = Odometry::new();
        self.slam = BasicSlam::new();
        self.navigator.reset();
        self.scan.reset();
        self.pose = Pose { x: 0.0, y: 0.0, theta: 0.0 };
        self.pose_initialized = false;
        self.min_front_dist = 10.0;

        let _ = self.gui_tx.send(GuiUpdate::ResetView);
        let _ = self.gui_tx.send(GuiUpdate::Pose { x: 0.0, y: 0.0, theta: 0.0 });
        let _ = self.gui_tx.send(GuiUpdate::SlamPose { x: 0.0, y: 0.0, theta: 0.0 });
        let _ = self.gui_tx.send(GuiUpdate::Encoders { left: 0, right: 0 });
        let _ = self.gui_tx.send(GuiUpdate::Map { width: 0, height: 0, data: vec![] });
        let _ = self.gui_tx.send(GuiUpdate::Frontiers(vec![]));
        let _ = self.gui_tx.send(GuiUpdate::Path(vec![]));
        let _ = self.gui_tx.send(GuiUpdate::Lidar(vec![]));
        let _ = self.gui_tx.send(GuiUpdate::NavigationTarget(None));
        self.stats.log("[SERVER] Reset completed.");
    }

    // ---- RPC -------------------------------------------------------------

    fn send_rpc(&mut self, method: &str, payload: Vec<u8>, reply: Option<mpsc::Sender<RpcResult>>) {
        let call_id = self.next_call_id;
        self.next_call_id = self.next_call_id.wrapping_add(1);
        self.pending_rpcs.insert(call_id, PendingRpc { method: method.to_string(), reply });
        self.stats.log(&format!("[RPC] -> {} (id {})", method, call_id));
        if let Err(e) = self.link.send(OutPayload::RpcRequest(crate::homerobot::RpcRequest {
            call_id,
            method: method.to_string(),
            payload,
        })) {
            log::error!("[RPC] Failed to send {}: {}", method, e);
            if let Some(pending) = self.pending_rpcs.remove(&call_id) {
                if let Some(reply) = pending.reply {
                    let _ = reply.send(Err(format!("send failed: {}", e)));
                }
            }
        }
    }

    fn handle_rpc_response(&mut self, resp: crate::homerobot::RpcResponse) {
        let result: RpcResult = if resp.error.is_empty() {
            Ok(())
        } else {
            Err(resp.error.clone())
        };
        let Some(pending) = self.pending_rpcs.remove(&resp.call_id) else {
            log::warn!("[RPC] Response for unknown call_id {} (error: '{}')", resp.call_id, resp.error);
            return;
        };
        match &result {
            Ok(()) => self.stats.log(&format!("[RPC] {} (id {}) completed successfully.", pending.method, resp.call_id)),
            Err(e) => self.stats.log(&format!("[RPC] {} (id {}) failed: {}", pending.method, resp.call_id, e)),
        }
        if let Some(reply) = pending.reply {
            let _ = reply.send(result);
        }
        if pending.method == "RunDiagnostic" && !resp.payload.is_empty() {
            if let Ok(diag) = <crate::homerobot::DiagnosticResult as Message>::decode(&*resp.payload) {
                self.stats.log(&format!("[DIAGNOSTICS] All OK: {}", diag.all_ok));
                for check in diag.checks {
                    self.stats.log(&format!(
                        "  - {}: {} ({})",
                        check.name,
                        if check.success { "PASS" } else { "FAIL" },
                        check.message
                    ));
                }
            }
        }
    }

    fn fail_pending_rpcs(&mut self, reason: &str) {
        for (_, pending) in self.pending_rpcs.drain() {
            if let Some(reply) = pending.reply {
                let _ = reply.send(Err(reason.to_string()));
            }
        }
    }

    // ---- Navigation ------------------------------------------------------

    fn run_navigation(&mut self) {
        if !self.pose_initialized {
            return;
        }
        let updates = self.navigator.tick(&self.slam, self.pose, self.min_front_dist);
        self.apply_nav_updates(updates);
    }

    fn apply_nav_updates(&mut self, updates: Vec<NavUpdate>) {
        for update in updates {
            match update {
                NavUpdate::Drive { left_power, left_angle, right_power, right_angle } => {
                    if let Err(e) = self.link.send_drive(left_power, left_angle, right_power, right_angle) {
                        log::error!("[NAV] Failed to send drive command: {}", e);
                    }
                }
                NavUpdate::Stop => {
                    if let Err(e) = self.link.stop() {
                        log::error!("[NAV] Failed to send stop: {}", e);
                    }
                }
                NavUpdate::PathChanged(path) => {
                    if !path.is_empty() {
                        let path_points: Vec<[f32; 3]> = path.iter().map(|p| [p.0, p.1, 0.05]).collect();
                        self.rec().log(
                            "robot/exploration/path",
                            &rerun::LineStrips3D::new([path_points])
                                .with_colors(vec![rerun::Color::from_rgb(0, 128, 255)]),
                        ).ok();
                    }
                    let _ = self.gui_tx.send(GuiUpdate::Path(path));
                }
                NavUpdate::GoalChanged(goal) => {
                    let _ = self.gui_tx.send(GuiUpdate::NavigationTarget(goal));
                }
                NavUpdate::FrontiersComputed(frontiers) => {
                    let frontier_points: Vec<[f32; 3]> =
                        frontiers.iter().map(|f| [f.centroid_x, f.centroid_y, 0.1]).collect();
                    self.rec().log(
                        "robot/exploration/frontiers",
                        &rerun::Points3D::new(frontier_points)
                            .with_colors(vec![rerun::Color::from_rgb(255, 255, 0)])
                            .with_radii(vec![0.05]),
                    ).ok();
                    let _ = self.gui_tx.send(GuiUpdate::Frontiers(frontiers));
                }
                NavUpdate::Finished { success } => {
                    self.stats.log(&format!(
                        "[NAV] Navigation finished ({}).",
                        if success { "goal reached" } else { "aborted" }
                    ));
                }
            }
        }
    }
}
