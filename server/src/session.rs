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
use crate::slam::Slam;
use crate::stats::Stats;
use crate::world::WorldModel;

/// Lidar is switched on as soon as a robot connects.
const DEFAULT_LIDAR_HZ: f32 = 5.0;
/// Receive timeout of the event loop; doubles as the navigation tick period
/// when no telemetry is arriving.
const EVENT_POLL_INTERVAL: Duration = Duration::from_millis(100);
const READ_TIMEOUT: Duration = Duration::from_millis(500);
/// How often the world model is checked for autosave to disk.
const AUTOSAVE_INTERVAL: Duration = Duration::from_secs(30);
/// Minimum interval between full occupancy-grid updates to the GUI: the grid
/// is ~720KB per message, far too heavy to clone on every 5Hz sweep.
const MAP_GUI_INTERVAL: Duration = Duration::from_millis(1000);

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

/// Per-sweep pose trace for offline SLAM evaluation, enabled with
/// `HR_POSE_LOG=<path>`. The column layout is a contract with
/// `tools/slam_benchmark.py` — extend, never reorder. `score` and `mode`
/// are placeholders until the SLAM engine exposes match health.
const POSE_TRACE_HEADER: &str = "t_unix,odom_x,odom_y,odom_theta,slam_x,slam_y,slam_theta,score,mode";

struct PoseTrace {
    out: Box<dyn Write + Send>,
}

impl PoseTrace {
    /// Opens the trace in append mode so a robot reconnect (new session)
    /// keeps extending the same run's file; the header is written only when
    /// the file is empty.
    fn from_env() -> Option<Self> {
        let path = std::env::var("HR_POSE_LOG").ok().filter(|p| !p.is_empty())?;
        match std::fs::OpenOptions::new().create(true).append(true).open(&path) {
            Ok(file) => {
                let is_empty = file.metadata().map(|m| m.len() == 0).unwrap_or(false);
                let mut trace = Self { out: Box::new(io::BufWriter::new(file)) };
                if is_empty {
                    trace.write_header();
                }
                log::info!("[POSE_LOG] Tracing SLAM poses to {}", path);
                Some(trace)
            }
            Err(e) => {
                log::error!("[POSE_LOG] Could not open {}: {}", path, e);
                None
            }
        }
    }

    #[cfg(test)]
    fn new(out: Box<dyn Write + Send>) -> Self {
        let mut trace = Self { out };
        trace.write_header();
        trace
    }

    fn write_header(&mut self) {
        let _ = writeln!(self.out, "{}", POSE_TRACE_HEADER);
    }

    fn record(&mut self, odom: &Pose, slam: &Pose, score: f32, mode: &str) {
        let t_unix = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0);
        let _ = writeln!(
            self.out,
            "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.4},{}",
            t_unix, odom.x, odom.y, odom.theta, slam.x, slam.y, slam.theta, score, mode
        );
        // Flushed per row: the benchmark tails the file live, and a killed
        // server (restart pattern P5) must not lose buffered rows.
        let _ = self.out.flush();
    }
}

struct PendingRpc {
    method: String,
    reply: Option<mpsc::Sender<RpcResult>>,
}

pub struct Session<W: Write> {
    link: RobotLink<W>,
    odom: Odometry,
    /// Shared across sessions: the map and pose survive robot reconnects.
    world: Arc<Mutex<WorldModel>>,
    navigator: Navigator,
    scan: ScanAssembler,
    min_front_dist: f32,
    pending_rpcs: HashMap<u32, PendingRpc>,
    next_call_id: u32,
    gui_tx: mpsc::Sender<GuiUpdate>,
    /// Cleared when the GUI receiver is gone (headless): skips building the
    /// heavy map updates entirely.
    gui_connected: bool,
    last_map_gui_update: Instant,
    map_gui_interval: Duration,
    stats: Arc<Stats>,
    rec: Arc<Mutex<rerun::RecordingStream>>,
    start_time: Instant,
    pose_trace: Option<PoseTrace>,
}

/// Entry point for a robot connection; returns when the connection drops or
/// the server shuts down.
#[allow(clippy::too_many_arguments)] // top-level wiring, called from one place
pub fn run_session(
    stream: TcpStream,
    bus: CommandBus,
    world: Arc<Mutex<WorldModel>>,
    generation: Arc<AtomicUsize>,
    stats: Arc<Stats>,
    sig_count: Arc<AtomicUsize>,
    gui_tx: mpsc::Sender<GuiUpdate>,
    rec: Arc<Mutex<rerun::RecordingStream>>,
) {
    // Single active robot: claiming a new generation retires every older
    // session, so two connections never write the shared world concurrently.
    let my_generation = generation.fetch_add(1, Ordering::SeqCst) + 1;
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
    // Kept aside to force-close the socket when this session ends, so the
    // reader thread unblocks and the robot notices immediately.
    let shutdown_handle = stream.try_clone().ok();
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

    world.lock().unwrap().on_new_session();
    let mut session = Session::new(stream, world, stats.clone(), gui_tx.clone(), rec);
    if let Err(e) = session.link.send(OutPayload::LidarControl(crate::homerobot::LidarControlCommand {
        active: true,
        target_frequency_hz: DEFAULT_LIDAR_HZ,
    })) {
        log::error!("[CONN] Failed to enable lidar on connect: {}", e);
    }

    while stats.running.load(Ordering::Relaxed) && sig_count.load(Ordering::Relaxed) == 0 {
        if generation.load(Ordering::SeqCst) != my_generation {
            stats.log(&format!("[CONN] Session for {} replaced by a newer connection.", addr));
            break;
        }
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
    if let Some(handle) = shutdown_handle {
        let _ = handle.shutdown(std::net::Shutdown::Both);
    }
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
    let mut protocol = ProtocolManager::new(stream);
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
        world: Arc<Mutex<WorldModel>>,
        stats: Arc<Stats>,
        gui_tx: mpsc::Sender<GuiUpdate>,
        rec: Arc<Mutex<rerun::RecordingStream>>,
    ) -> Self {
        let start_time = Instant::now();
        Self {
            link: RobotLink {
                stream,
                start_time,
                last_drive: None,
            },
            odom: Odometry::new(),
            world,
            navigator: Navigator::new(),
            scan: ScanAssembler::new(),
            min_front_dist: 10.0,
            pending_rpcs: HashMap::new(),
            next_call_id: 1,
            gui_tx,
            gui_connected: true,
            last_map_gui_update: Instant::now() - MAP_GUI_INTERVAL,
            map_gui_interval: MAP_GUI_INTERVAL,
            stats,
            rec,
            start_time,
            pose_trace: PoseTrace::from_env(),
        }
    }

    #[cfg(test)]
    fn set_map_gui_interval(&mut self, interval: Duration) {
        self.map_gui_interval = interval;
    }

    #[cfg(test)]
    fn set_pose_trace(&mut self, out: Box<dyn Write + Send>) {
        self.pose_trace = Some(PoseTrace::new(out));
    }

    /// Rerun stream with the session timeline set to "now".
    fn rec(&self) -> rerun::RecordingStream {
        let rec = self.rec.lock().unwrap().clone();
        rec.set_time("realtime", self.start_time.elapsed());
        rec
    }

    pub fn handle_event(&mut self, event: SessionEvent) {
        // The session is the only writer while it lives; one lock per event.
        let world = self.world.clone();
        let mut world = world.lock().unwrap();
        match event {
            SessionEvent::FromRobot(msg) => {
                let seq = msg.sequence_millis;
                if let Some(payload) = msg.payload {
                    self.handle_robot_payload(&mut world, payload, seq);
                }
            }
            SessionEvent::Command(cmd) => self.handle_command(&mut world, cmd),
            SessionEvent::Tick => self.run_navigation(&mut world),
            SessionEvent::Disconnected(_) => unreachable!("handled by the session loop"),
        }
        // Checked on every event, not just Tick: under continuous telemetry
        // the receive timeout never fires, so Tick events are starved.
        self.maybe_autosave(&mut world);
    }

    // ---- Robot telemetry -------------------------------------------------

    fn handle_robot_payload(&mut self, world: &mut WorldModel, payload: Payload, sequence_millis: u32) {
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
                self.handle_encoders(world, enc.left_encoder, enc.right_encoder, sequence_millis);
                self.run_navigation(world);
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
                self.run_navigation(world);
            }
            Payload::Lidar(scan) => {
                for sweep in self.scan.push(scan.points) {
                    self.process_sweep(world, sweep);
                }
                self.run_navigation(world);
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

    fn handle_encoders(&mut self, world: &mut WorldModel, left: i32, right: i32, sequence_millis: u32) {
        if left != 0 || right != 0 {
            log::info!("[SERVER] Telemetry Heartbeat: Encoders delta L={} R={}", left, right);
        }
        let old_odom_pose = self.odom.pose;
        self.odom.update_encoders(left, right, sequence_millis);
        world.slam.add_odom_pose(self.odom.pose, Instant::now());

        if !world.pose_initialized {
            world.pose = self.odom.pose;
            world.pose_initialized = true;
        } else {
            // Propagate the odometry delta into the SLAM-corrected pose so the
            // path follower doesn't act on stale position between sweeps.
            let dx = self.odom.pose.x - old_odom_pose.x;
            let dy = self.odom.pose.y - old_odom_pose.y;
            let dt = self.odom.pose.theta - old_odom_pose.theta;

            // Rotate the translation delta into the corrected pose's frame.
            let diff_theta = world.pose.theta - old_odom_pose.theta;
            let (sin_diff, cos_diff) = diff_theta.sin_cos();
            world.pose.x += dx * cos_diff - dy * sin_diff;
            world.pose.y += dx * sin_diff + dy * cos_diff;
            world.pose.theta += dt;
            while world.pose.theta > std::f32::consts::PI {
                world.pose.theta -= 2.0 * std::f32::consts::PI;
            }
            while world.pose.theta < -std::f32::consts::PI {
                world.pose.theta += 2.0 * std::f32::consts::PI;
            }

            let _ = self.gui_tx.send(GuiUpdate::SlamPose {
                x: world.pose.x,
                y: world.pose.y,
                theta: world.pose.theta,
            });
            self.log_slam_pose(world.pose);
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

    fn process_sweep(&mut self, world: &mut WorldModel, sweep: CompletedSweep) {
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
        world.pose = world.slam.update(&sweep.points, &self.odom.pose);
        world.pose_initialized = true;
        world.mark_dirty();
        if let Some(trace) = &mut self.pose_trace {
            // score/mode are placeholders until Slam::health() exists (T11).
            trace.record(&self.odom.pose, &world.pose, 0.0, "basic");
        }
        let _ = self.gui_tx.send(GuiUpdate::Lidar(sweep.points.clone()));
        let _ = self.gui_tx.send(GuiUpdate::SlamPose {
            x: world.pose.x,
            y: world.pose.y,
            theta: world.pose.theta,
        });

        let rec = self.rec();
        self.log_slam_pose(world.pose);
        let (rerun_pts, rerun_colors): (Vec<[f32; 3]>, Vec<rerun::Color>) = sweep.points.iter().map(|pt| {
            let angle = -pt.angle_deg.to_radians();
            let d = pt.distance_mm / 1000.0;
            let t = (pt.quality as f32 / 15.0).clamp(0.0, 1.0);
            let r = ((1.0 - t) * 255.0) as u8;
            let g = (t * 255.0) as u8;
            ([d * angle.cos(), d * angle.sin(), 0.0], rerun::Color::from_rgb(r, g, 0))
        }).unzip();
        rec.log("robot/sensor/lidar", &rerun::Points3D::new(rerun_pts).with_colors(rerun_colors)).ok();

        // The full grid clone is heavy; throttle it and skip it entirely once
        // the GUI receiver is gone (headless mode).
        if self.gui_connected && self.last_map_gui_update.elapsed() >= self.map_gui_interval {
            self.last_map_gui_update = Instant::now();
            let (w, h, data) = world.slam.get_map_data();
            if self.gui_tx.send(GuiUpdate::Map {
                width: w,
                height: h,
                data: data.to_vec(),
            }).is_err() {
                self.gui_connected = false;
            }
        }
    }

    fn log_slam_pose(&self, pose: Pose) {
        self.rec().log(
            "robot/pose/slam",
            &rerun::Transform3D::from_translation_rotation(
                [pose.x, pose.y, 0.0],
                rerun::RotationAxisAngle::new([0.0, 0.0, 1.0], rerun::Angle::from_radians(pose.theta)),
            ),
        ).ok();
    }

    // ---- Commands --------------------------------------------------------

    fn handle_command(&mut self, world: &mut WorldModel, cmd: Command) {
        match cmd {
            Command::Drive { left_power, left_angle, right_power, right_angle } => {
                self.enter_manual(world.pose);
                if let Err(e) = self.link.send_drive(left_power, left_angle, right_power, right_angle) {
                    log::error!("[CMD] Failed to send drive command: {}", e);
                }
            }
            Command::StopMoving => {
                self.enter_manual(world.pose);
                if let Err(e) = self.link.stop() {
                    log::error!("[CMD] Failed to send stop: {}", e);
                }
            }
            Command::StopAll => {
                self.enter_manual(world.pose);
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
                self.enter_manual(world.pose);
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
                match world.slam.save_map(map_name) {
                    Ok(()) => self.stats.log(&format!("[SLAM] Map saved successfully to {}!", abs_path.display())),
                    Err(e) => log::error!("Error saving map: {:?}", e),
                }
            }
            Command::Reset => self.reset(world),
            Command::SetMode(mode) => {
                let updates = self.navigator.set_mode(mode, world.pose);
                self.apply_nav_updates(updates);
                // Plan immediately instead of waiting for the next telemetry event.
                self.run_navigation(world);
            }
        }
    }

    /// Persist the world periodically so it survives server restarts.
    fn maybe_autosave(&mut self, world: &mut WorldModel) {
        if !world.autosave_due(AUTOSAVE_INTERVAL) {
            return;
        }
        match world.save(crate::world::AUTOSAVE_PATH) {
            Ok(()) => self.stats.log(&format!("[WORLD] Map autosaved to {}", crate::world::AUTOSAVE_PATH)),
            Err(e) => log::error!("[WORLD] Autosave failed: {}", e),
        }
    }

    /// Any manual command cancels autonomous navigation.
    fn enter_manual(&mut self, pose: Pose) {
        let updates = self.navigator.set_mode(NavMode::Manual, pose);
        self.apply_nav_updates(updates);
    }

    fn reset(&mut self, world: &mut WorldModel) {
        self.stats.log("[SERVER] Resetting map, path, and odometry...");
        let _ = self.link.stop();
        self.odom = Odometry::new();
        world.reset();
        self.navigator.reset();
        self.scan.reset();
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

    fn run_navigation(&mut self, world: &mut WorldModel) {
        if !world.pose_initialized {
            return;
        }
        let updates = self.navigator.tick(&world.slam, world.pose, self.min_front_dist);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::homerobot::{MotorEncoders, RpcResponse};

    fn test_session() -> (Session<Vec<u8>>, mpsc::Receiver<GuiUpdate>) {
        test_session_with_world(Arc::new(Mutex::new(WorldModel::new())))
    }

    fn test_session_with_world(world: Arc<Mutex<WorldModel>>) -> (Session<Vec<u8>>, mpsc::Receiver<GuiUpdate>) {
        let (gui_tx, gui_rx) = mpsc::channel();
        world.lock().unwrap().on_new_session();
        let session = Session::new(
            Vec::new(),
            world,
            Stats::new(),
            gui_tx,
            Arc::new(Mutex::new(rerun::RecordingStream::disabled())),
        );
        (session, gui_rx)
    }

    fn world_of(session: &Session<Vec<u8>>) -> Arc<Mutex<WorldModel>> {
        session.world.clone()
    }

    /// Decode every length-prefixed frame the session wrote to the wire.
    fn sent_messages(session: &Session<Vec<u8>>) -> Vec<ServerToRobotMessage> {
        let mut buf = session.link.stream.as_slice();
        let mut messages = Vec::new();
        while buf.len() >= 2 {
            let len = u16::from_be_bytes([buf[0], buf[1]]) as usize;
            messages.push(ServerToRobotMessage::decode(&buf[2..2 + len]).expect("valid frame"));
            buf = &buf[2 + len..];
        }
        messages
    }

    fn encoders_event(left: i32, right: i32, millis: u32) -> SessionEvent {
        SessionEvent::FromRobot(RobotToServerMessage {
            sequence_millis: millis,
            payload: Some(Payload::Encoders(MotorEncoders {
                left_encoder: left,
                right_encoder: right,
            })),
        })
    }

    /// A complete 41-point lidar revolution; the trailing `scan_completed`
    /// marker flushes the buffered sweep through the assembler.
    fn sweep_event(offset: f32) -> SessionEvent {
        let mut points: Vec<crate::homerobot::LidarPoint> = (0..40)
            .map(|i| crate::homerobot::LidarPoint {
                distance_mm: 1000.0,
                angle_deg: offset + i as f32 * 9.0,
                quality: 15,
                scan_completed: false,
            })
            .collect();
        points.push(crate::homerobot::LidarPoint {
            distance_mm: 1000.0,
            angle_deg: offset,
            quality: 15,
            scan_completed: true,
        });
        SessionEvent::FromRobot(RobotToServerMessage {
            sequence_millis: 0,
            payload: Some(Payload::Lidar(crate::homerobot::LidarScan { points })),
        })
    }

    #[test]
    fn encoder_telemetry_updates_pose_and_notifies_gui() {
        let (mut session, gui_rx) = test_session();
        // ~1m forward at the default 1736.2 ticks/m.
        session.handle_event(encoders_event(1736, 1736, 100));

        let world = world_of(&session);
        let world = world.lock().unwrap();
        assert!(world.pose_initialized);
        assert!((world.pose.x - 1.0).abs() < 0.01);
        drop(world);

        let updates: Vec<GuiUpdate> = gui_rx.try_iter().collect();
        assert!(updates.iter().any(|u| matches!(u, GuiUpdate::Encoders { left: 1736, right: 1736 })));
        assert!(updates.iter().any(|u| matches!(u, GuiUpdate::Pose { x, .. } if (x - 1.0).abs() < 0.01)));
    }

    #[test]
    fn execute_motion_rpc_completes_on_matching_call_id() {
        let (mut session, _gui_rx) = test_session();
        let (reply_tx, reply_rx) = mpsc::channel();
        session.handle_event(SessionEvent::Command(Command::ExecuteMotion {
            motion_type: 0,
            left_ticks: 100,
            right_ticks: 100,
            max_power: 150,
            reply: Some(reply_tx),
        }));

        let sent = sent_messages(&session);
        let call_id = sent
            .iter()
            .find_map(|m| match &m.payload {
                Some(OutPayload::RpcRequest(r)) => Some(r.call_id),
                _ => None,
            })
            .expect("RpcRequest was sent");

        session.handle_event(SessionEvent::FromRobot(RobotToServerMessage {
            sequence_millis: 0,
            payload: Some(Payload::RpcResponse(RpcResponse {
                call_id,
                payload: vec![],
                error: String::new(),
            })),
        }));

        assert_eq!(reply_rx.try_recv(), Ok(Ok(())));
    }

    #[test]
    fn rpc_response_with_unknown_call_id_completes_nothing() {
        let (mut session, _gui_rx) = test_session();
        let (reply_tx, reply_rx) = mpsc::channel();
        session.handle_event(SessionEvent::Command(Command::ExecuteMotion {
            motion_type: 0,
            left_ticks: 100,
            right_ticks: 100,
            max_power: 150,
            reply: Some(reply_tx),
        }));

        session.handle_event(SessionEvent::FromRobot(RobotToServerMessage {
            sequence_millis: 0,
            payload: Some(Payload::RpcResponse(RpcResponse {
                call_id: 9999,
                payload: vec![],
                error: String::new(),
            })),
        }));

        assert!(reply_rx.try_recv().is_err(), "stale response must not complete a pending motion");
    }

    #[test]
    fn identical_drive_commands_are_deduped_on_the_wire() {
        let (mut session, _gui_rx) = test_session();
        let drive = || Command::Drive { left_power: 100, left_angle: 1.0, right_power: 100, right_angle: 1.0 };
        session.handle_event(SessionEvent::Command(drive()));
        session.handle_event(SessionEvent::Command(drive()));

        let motor_moves = sent_messages(&session)
            .iter()
            .filter(|m| matches!(m.payload, Some(OutPayload::MotorMove(_))))
            .count();
        assert_eq!(motor_moves, 1);
    }

    #[test]
    fn manual_drive_cancels_autonomous_mode_and_stops_first() {
        let (mut session, gui_rx) = test_session();
        session.handle_event(encoders_event(10, 10, 100));
        session.handle_event(SessionEvent::Command(Command::SetMode(NavMode::Exploration)));
        let _ = gui_rx.try_iter().count(); // drain

        session.handle_event(SessionEvent::Command(Command::Drive {
            left_power: 80, left_angle: 1.0, right_power: 80, right_angle: 1.0,
        }));

        // Leaving Exploration emits a stop, then the manual drive follows.
        let moves: Vec<u32> = sent_messages(&session)
            .iter()
            .filter_map(|m| match &m.payload {
                Some(OutPayload::MotorMove(mv)) => Some(mv.left_power),
                _ => None,
            })
            .collect();
        assert_eq!(moves.last(), Some(&80));
        assert!(moves.contains(&0), "expected a stop when leaving autonomous mode");

        let updates: Vec<GuiUpdate> = gui_rx.try_iter().collect();
        assert!(updates.iter().any(|u| matches!(u, GuiUpdate::NavigationTarget(None))));
    }

    #[test]
    fn map_gui_updates_are_throttled() {
        let (mut session, gui_rx) = test_session();
        session.set_map_gui_interval(Duration::from_millis(400));

        // Sweeps must be >80ms apart for the assembler, but within the map
        // throttle window: expect exactly one Map update for the pair.
        session.handle_event(sweep_event(0.0));
        std::thread::sleep(Duration::from_millis(100));
        session.handle_event(sweep_event(0.5));
        let maps = gui_rx.try_iter().filter(|u| matches!(u, GuiUpdate::Map { .. })).count();
        assert_eq!(maps, 1);

        // After the throttle interval a new sweep publishes the map again.
        std::thread::sleep(Duration::from_millis(350));
        session.handle_event(sweep_event(1.0));
        let maps = gui_rx.try_iter().filter(|u| matches!(u, GuiUpdate::Map { .. })).count();
        assert_eq!(maps, 1);
    }

    #[test]
    fn pose_trace_writes_header_and_one_csv_row_per_sweep() {
        /// Cloneable sink so the test keeps a handle to what the trace wrote.
        #[derive(Clone, Default)]
        struct SharedBuf(Arc<Mutex<Vec<u8>>>);
        impl Write for SharedBuf {
            fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
                self.0.lock().unwrap().extend_from_slice(buf);
                Ok(buf.len())
            }
            fn flush(&mut self) -> io::Result<()> {
                Ok(())
            }
        }

        let (mut session, _gui_rx) = test_session();
        let buf = SharedBuf::default();
        session.set_pose_trace(Box::new(buf.clone()));

        session.handle_event(encoders_event(100, 100, 100));
        session.handle_event(sweep_event(0.0));

        let text = String::from_utf8(buf.0.lock().unwrap().clone()).unwrap();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines[0], POSE_TRACE_HEADER);
        assert_eq!(lines.len(), 2, "exactly one data row per processed sweep");

        let cols: Vec<&str> = lines[1].split(',').collect();
        assert_eq!(cols.len(), POSE_TRACE_HEADER.split(',').count());
        for col in &cols[..8] {
            col.parse::<f64>().expect("numeric columns must parse");
        }
        assert_eq!(cols[8], "basic");
    }

    #[test]
    fn reset_zeroes_state_and_emits_reset_view() {
        let (mut session, gui_rx) = test_session();
        session.handle_event(encoders_event(1736, 1736, 100));
        assert!(session.world.lock().unwrap().pose_initialized);
        let _ = gui_rx.try_iter().count(); // drain

        session.handle_event(SessionEvent::Command(Command::Reset));

        let world = world_of(&session);
        let world = world.lock().unwrap();
        assert!(!world.pose_initialized);
        assert_eq!(world.pose.x, 0.0);
        drop(world);
        let updates: Vec<GuiUpdate> = gui_rx.try_iter().collect();
        assert!(updates.iter().any(|u| matches!(u, GuiUpdate::ResetView)));
        assert!(updates.iter().any(|u| matches!(u, GuiUpdate::Map { width: 0, height: 0, .. })));
    }

    #[test]
    fn world_state_survives_session_replacement() {
        let world = Arc::new(Mutex::new(WorldModel::new()));

        // First session drives ~1m forward, then the connection drops.
        let (mut first, _gui1) = test_session_with_world(world.clone());
        first.handle_event(encoders_event(1736, 1736, 100));
        assert!((world.lock().unwrap().pose.x - 1.0).abs() < 0.01);
        drop(first);

        // The replacement session sees the same pose and map...
        let (mut second, _gui2) = test_session_with_world(world.clone());
        {
            let w = world.lock().unwrap();
            assert!(w.pose_initialized, "pose must survive the reconnect");
            assert!((w.pose.x - 1.0).abs() < 0.01);
        }

        // ...and new odometry (restarting at zero) extends it instead of
        // teleporting the robot back to the origin.
        second.handle_event(encoders_event(1736, 1736, 100));
        assert!((world.lock().unwrap().pose.x - 2.0).abs() < 0.02);
    }

    #[test]
    fn failed_send_reports_error_to_rpc_caller() {
        // A session over a full/broken sink: use a writer that always errors.
        struct BrokenSink;
        impl Write for BrokenSink {
            fn write(&mut self, _buf: &[u8]) -> io::Result<usize> {
                Err(io::Error::new(io::ErrorKind::BrokenPipe, "gone"))
            }
            fn flush(&mut self) -> io::Result<()> {
                Ok(())
            }
        }

        let (gui_tx, _gui_rx) = mpsc::channel();
        let mut session = Session::new(
            BrokenSink,
            Arc::new(Mutex::new(WorldModel::new())),
            Stats::new(),
            gui_tx,
            Arc::new(Mutex::new(rerun::RecordingStream::disabled())),
        );
        let (reply_tx, reply_rx) = mpsc::channel();
        session.handle_event(SessionEvent::Command(Command::ExecuteMotion {
            motion_type: 0,
            left_ticks: 100,
            right_ticks: 100,
            max_power: 150,
            reply: Some(reply_tx),
        }));

        assert!(matches!(reply_rx.try_recv(), Ok(Err(_))), "send failure must fail the RPC immediately");
        assert!(session.pending_rpcs.is_empty());
    }
}
