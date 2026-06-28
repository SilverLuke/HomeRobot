mod reader;
mod sender;
mod constants;
mod stats;
mod gui;
mod command;
mod connection;
mod odometry;
mod slam;
mod mapping;
mod pathfinding;

pub mod homerobot {
    include!(concat!(env!("OUT_DIR"), "/homerobot.rs"));
}

use gtk4::prelude::*;
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::thread::sleep;
use std::time::Duration;
use std::{io, thread, process, env};
use std::io::{Write, Read};

use crate::command::RobotCommand;
use crate::stats::Stats;
use crate::gui::init_gui;
use crate::connection::handle_connection;
use prost::Message;

fn handle_proxy_client(mut stream: TcpStream, robot_command: Arc<Mutex<RobotCommand>>) {
    log::info!("[PROXY] cmd_sender connected.");
    stream.set_read_timeout(Some(Duration::from_millis(500))).ok();
    
    loop {
        let mut len_buf = [0u8; 2];
        match stream.read_exact(&mut len_buf) {
            Ok(_) => {
                let len = u16::from_be_bytes(len_buf) as usize;
                let mut payload_buf = vec![0u8; len];
                if stream.read_exact(&mut payload_buf).is_ok() {
                    if let Ok(msg) = homerobot::ServerToRobotMessage::decode(&*payload_buf) {
                        if let Some(payload) = msg.payload {
                            if let Ok(mut cmd) = robot_command.lock() {
                                match payload {
                                    homerobot::server_to_robot_message::Payload::MotorMove(m) => {
                                        *cmd = RobotCommand::MotorAngle {
                                            left_power: m.left_power as u8,
                                            left_angle: m.left_angle,
                                            right_power: m.right_power as u8,
                                            right_angle: m.right_angle,
                                        };
                                    }
                                    homerobot::server_to_robot_message::Payload::StopAll(_) => {
                                        *cmd = RobotCommand::StopAll;
                                    }
                                    homerobot::server_to_robot_message::Payload::LidarControl(l) => {
                                        *cmd = RobotCommand::LidarControl {
                                            active: l.active,
                                            target_frequency_hz: l.target_frequency_hz,
                                        };
                                    }
                                    homerobot::server_to_robot_message::Payload::RpcRequest(r) => {
                                        log::info!("[PROXY] Received RPC Request: {}", r.method);
                                         if r.method == "RunDiagnostic" {
                                             *cmd = RobotCommand::RunDiagnostic;
                                             log::info!("[PROXY] Forwarding RunDiagnostic");
                                         } else if r.method == "SaveMap" {
                                             *cmd = RobotCommand::SaveMap;
                                             log::info!("[PROXY] Triggering SaveMap");
                                         } else if r.method == "StartExplore" {
                                             *cmd = RobotCommand::AutonomousExploration { enabled: true };
                                             log::info!("[PROXY] Starting Autonomous Exploration");
                                         } else if r.method == "StopExplore" {
                                             *cmd = RobotCommand::AutonomousExploration { enabled: false };
                                             log::info!("[PROXY] Stopping Autonomous Exploration");
                                         } else if r.method == "NavigateTo" {
                                             if r.payload.len() >= 8 {
                                                 let x = f32::from_le_bytes(r.payload[0..4].try_into().unwrap());
                                                 let y = f32::from_le_bytes(r.payload[4..8].try_into().unwrap());
                                                 *cmd = RobotCommand::NavigateTo { x, y };
                                                 log::info!("[PROXY] Starting Navigation to X={:.2}, Y={:.2}", x, y);
                                             } else {
                                                 log::warn!("[PROXY] Invalid payload for NavigateTo");
                                             }
                                         } else if r.method == "ExecuteMotion" {
                                             if let Ok(motion_req) = <homerobot::MotionRequest as prost::Message>::decode(&*r.payload) {
                                                 let (ticks_per_meter, wheel_base) = {
                                                     let sizes = crate::constants::ROBOT_SIZES.lock().unwrap();
                                                     (sizes.ticks_per_meter, sizes.wheel_base)
                                                 };
                                                 
                                                 let (left_ticks, right_ticks) = match motion_req.r#type {
                                                     0 => { // STRAIGHT
                                                         let ticks = (motion_req.distance * ticks_per_meter) as i32;
                                                         (ticks, ticks)
                                                     }
                                                     1 => { // ROTATE
                                                         let angle_rad = motion_req.angle.to_radians();
                                                         let dist = angle_rad * (wheel_base / 2.0);
                                                         let ticks = (dist * ticks_per_meter) as i32;
                                                         (-ticks, ticks)
                                                     }
                                                     2 => { // ARC
                                                         let angle_rad = motion_req.angle.to_radians();
                                                         let left_dist = angle_rad * (motion_req.radius - wheel_base / 2.0);
                                                         let right_dist = angle_rad * (motion_req.radius + wheel_base / 2.0);
                                                         ((left_dist * ticks_per_meter) as i32, (right_dist * ticks_per_meter) as i32)
                                                     }
                                                     _ => (motion_req.left_ticks, motion_req.right_ticks),
                                                 };
                                                 
                                                 *cmd = RobotCommand::ExecuteMotion {
                                                     motion_type: motion_req.r#type,
                                                     left_ticks,
                                                     right_ticks,
                                                     max_power: motion_req.max_power,
                                                 };
                                                 log::info!("[PROXY] ExecuteMotion calculated: L={} R={}", left_ticks, right_ticks);
                                                 
                                                 drop(cmd);
                                                 
                                                 let &(ref lock, ref cvar) = &**crate::connection::MOTION_COMPLETED;
                                                 {
                                                     let mut completed = lock.lock().unwrap();
                                                     *completed = None;
                                                 }
                                                 
                                                 let mut completed = lock.lock().unwrap();
                                                 while completed.is_none() {
                                                     completed = cvar.wait(completed).unwrap();
                                                 }
                                                 
                                                 match completed.as_ref().unwrap() {
                                                     Ok(()) => {
                                                         log::info!("[PROXY] Motion completed successfully.");
                                                         let _ = stream.write_all(&[1u8]);
                                                     }
                                                     Err(e) => {
                                                         log::error!("[PROXY] Motion failed: {}", e);
                                                         let _ = stream.write_all(&[0u8]);
                                                     }
                                                 }
                                                 break;
                                             }
                                         }
                                    }
                                    _ => {
                                        log::warn!("[PROXY] Received unknown payload type");
                                    }
                                }
                            }
                        } else {
                            log::warn!("[PROXY] Received message with no payload");
                        }
                    } else {
                        log::error!("[PROXY] Failed to decode Protobuf message");
                    }
                }
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => { continue; }
            Err(_) => {
                log::info!("[PROXY] cmd_sender disconnected.");
                break;
            }
        }
    }
}

fn main() -> io::Result<()> {
    // Initialize logger
    if std::env::var("RUST_LOG").is_err() {
        std::env::set_var("RUST_LOG", "info");
    }
    pretty_env_logger::init();

    // Initialize Rerun (save to file in HEADLESS mode, disabled otherwise)
    let is_headless = env::var("HEADLESS").is_ok();
    
    let rec = if is_headless {
        let stream = rerun::RecordingStreamBuilder::new("home-robot")
            .save("logs/sim/simulation.rrd")
            .expect("Could not initialize Rerun to save to file");
        Arc::new(Mutex::new(stream))
    } else {
        Arc::new(Mutex::new(rerun::RecordingStream::disabled()))
    };

    let stats = Stats::new();
    let robot_command = Arc::new(Mutex::new(RobotCommand::LidarControl {
        active: true,
        target_frequency_hz: 5.0,
    }));
    let sig_count = Arc::new(AtomicUsize::new(0));

    // 1. Initialize GTK GUI (only if not headless)
    let (gui_app, gui_tx) = if !is_headless {
        let (app, tx) = init_gui(robot_command.clone(), rec.clone());
        (Some(app), tx)
    } else {
        let (tx, _rx) = std::sync::mpsc::channel();
        (None, tx)
    };

    // 2. Signal Handling
    let sc = sig_count.clone();
    let stats_signal = stats.clone();
    ctrlc::set_handler(move || {
        let count = sc.fetch_add(1, Ordering::SeqCst) + 1;
        if count == 1 {
            stats_signal.running.store(false, Ordering::SeqCst);
            let _ = writeln!(io::stderr(), "\r\n[CTRL+C] Shutdown initiated. Press again to force exit.\r");
        } else {
            let _ = writeln!(io::stderr(), "\r\n[CTRL+C] Force exit requested.\r");
            process::exit(0);
        }
    }).expect("Error setting Ctrl-C handler");

    println!("======================================\r");
    println!("      Robot Control Server v1.0       \r");
    if is_headless {
        println!("      (HEADLESS MODE ACTIVE)          \r");
    } else {
        println!("      (with GTK4 Dashboard)           \r");
    }
    println!("======================================\r");

    // 3. Start Listener
    let stats_server = stats.clone();
    let rc_server = robot_command.clone();
    let sig_count_server = sig_count.clone();
    let gui_tx_server = gui_tx.clone();
    let rec_server = rec.clone();
    thread::spawn(move || {
        let addr = "0.0.0.0:12345";
        let listener = TcpListener::bind(addr).expect("Could not bind");
        listener.set_nonblocking(true).ok();
        log::info!("[SERVER] Listening on {}...", addr);

        while stats_server.running.load(Ordering::Relaxed) && sig_count_server.load(Ordering::Relaxed) == 0 {
            match listener.accept() {
                Ok((stream, _)) => {
                    let rc = Arc::clone(&rc_server);
                    let st = Arc::clone(&stats_server);
                    let sc = Arc::clone(&sig_count_server);
                    let gtx = gui_tx_server.clone();
                    let rrec = rec_server.clone();
                    thread::spawn(move || handle_connection(stream, rc, st, sc, gtx, rrec));
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                    sleep(Duration::from_millis(100));
                }
                _ => {}
            }
        }
    });

    // 4. Start Proxy Listener for cmd_sender
    let rc_proxy = robot_command.clone();
    let stats_proxy = stats.clone();
    thread::spawn(move || {
        let addr = "0.0.0.0:12346";
        let listener = TcpListener::bind(addr).expect("Could not bind Proxy");
        listener.set_nonblocking(true).ok();
        log::info!("[PROXY] Listening for cmd_sender on {}...", addr);

        while stats_proxy.running.load(Ordering::Relaxed) {
            match listener.accept() {
                Ok((stream, _)) => {
                    let rc = Arc::clone(&rc_proxy);
                    thread::spawn(move || handle_proxy_client(stream, rc));
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                    sleep(Duration::from_millis(100));
                }
                _ => {}
            }
        }
    });

    // 5. Start UDP Syslog listener
    let stats_syslog = stats.clone();
    let gui_tx_syslog = gui_tx.clone();
    thread::spawn(move || {
        use std::net::UdpSocket;
        let socket = match UdpSocket::bind("0.0.0.0:5140") {
            Ok(s) => s,
            Err(e) => {
                log::error!("[SYSLOG ERROR] Could not bind to port 5140: {}", e);
                return;
            }
        };
        socket.set_read_timeout(Some(Duration::from_millis(500))).ok();
        log::info!("[SYSLOG] Listening on UDP 0.0.0.0:5140");

        let mut buf = [0u8; 1024];
        while stats_syslog.running.load(Ordering::Relaxed) {
            match socket.recv_from(&mut buf) {
                Ok((amt, _src)) => {
                    if let Ok(msg) = std::str::from_utf8(&buf[..amt]) {
                        let clean_msg = msg.trim();
                        if is_headless {
                            log::info!("[ROBOT] {}", clean_msg);
                        }
                        let _ = gui_tx_syslog.send(crate::gui::GuiUpdate::Log(clean_msg.to_string()));
                    }
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                    continue;
                }
                Err(_) => {}
            }
        }
    });

    // 6. Run GTK GUI or Headless Loop
    if is_headless {
        log::info!("[SERVER] Running in background. Waiting for connections...");
        while stats.running.load(Ordering::Relaxed) {
            sleep(Duration::from_millis(100));
        }
    } else if let Some(app) = gui_app {
        app.run();
    }

    println!("\n\r[SHUTDOWN] Exiting...\r");
    Ok(())
}
