mod reader;
mod constants;
mod stats;
mod gui;
mod command;
mod navigator;
mod scan;
mod session;
mod world;
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

use crate::command::{Command, CommandBus, NavMode};
use crate::stats::Stats;
use crate::gui::init_gui;
use crate::session::run_session;
use prost::Message;

/// Translates the external cmd_sender protocol into bus commands.
fn handle_proxy_client(mut stream: TcpStream, bus: CommandBus) {
    log::info!("[PROXY] cmd_sender connected.");
    stream.set_read_timeout(Some(Duration::from_millis(500))).ok();

    loop {
        let mut len_buf = [0u8; 2];
        match stream.read_exact(&mut len_buf) {
            Ok(_) => {
                let len = u16::from_be_bytes(len_buf) as usize;
                let mut payload_buf = vec![0u8; len];
                if stream.read_exact(&mut payload_buf).is_ok() {
                    match homerobot::ServerToRobotMessage::decode(&*payload_buf) {
                        Ok(msg) => {
                            if let Some(payload) = msg.payload {
                                // ExecuteMotion answers on this stream and then closes it.
                                if handle_proxy_payload(payload, &bus, &mut stream) {
                                    break;
                                }
                            } else {
                                log::warn!("[PROXY] Received message with no payload");
                            }
                        }
                        Err(_) => log::error!("[PROXY] Failed to decode Protobuf message"),
                    }
                }
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => continue,
            Err(_) => {
                log::info!("[PROXY] cmd_sender disconnected.");
                break;
            }
        }
    }
}

/// Returns true when the proxy connection should be closed.
fn handle_proxy_payload(
    payload: homerobot::server_to_robot_message::Payload,
    bus: &CommandBus,
    stream: &mut TcpStream,
) -> bool {
    use homerobot::server_to_robot_message::Payload;
    match payload {
        Payload::MotorMove(m) => {
            bus.send(Command::Drive {
                left_power: m.left_power as u8,
                left_angle: m.left_angle,
                right_power: m.right_power as u8,
                right_angle: m.right_angle,
            });
        }
        Payload::StopAll(_) => bus.send(Command::StopAll),
        Payload::LidarControl(l) => {
            bus.send(Command::LidarControl {
                active: l.active,
                target_frequency_hz: l.target_frequency_hz,
            });
        }
        Payload::RpcRequest(r) => {
            log::info!("[PROXY] Received RPC Request: {}", r.method);
            match r.method.as_str() {
                "RunDiagnostic" => bus.send(Command::RunDiagnostic),
                "SaveMap" => bus.send(Command::SaveMap),
                "StartExplore" => bus.send(Command::SetMode(NavMode::Exploration)),
                "StopExplore" => bus.send(Command::SetMode(NavMode::Manual)),
                "NavigateTo" => {
                    if r.payload.len() >= 8 {
                        let x = f32::from_le_bytes(r.payload[0..4].try_into().unwrap());
                        let y = f32::from_le_bytes(r.payload[4..8].try_into().unwrap());
                        log::info!("[PROXY] Starting Navigation to X={:.2}, Y={:.2}", x, y);
                        bus.send(Command::SetMode(NavMode::NavigateTo { x, y }));
                    } else {
                        log::warn!("[PROXY] Invalid payload for NavigateTo");
                    }
                }
                "ExecuteMotion" => {
                    if let Ok(motion_req) = <homerobot::MotionRequest as prost::Message>::decode(&*r.payload) {
                        let (left_ticks, right_ticks) = motion_ticks(&motion_req);
                        log::info!("[PROXY] ExecuteMotion calculated: L={} R={}", left_ticks, right_ticks);

                        let (reply_tx, reply_rx) = std::sync::mpsc::channel();
                        bus.send(Command::ExecuteMotion {
                            motion_type: motion_req.r#type,
                            left_ticks,
                            right_ticks,
                            max_power: motion_req.max_power,
                            reply: Some(reply_tx),
                        });

                        let result = reply_rx
                            .recv_timeout(Duration::from_secs(120))
                            .unwrap_or_else(|_| Err("timeout waiting for motion completion".to_string()));
                        match result {
                            Ok(()) => {
                                log::info!("[PROXY] Motion completed successfully.");
                                let _ = stream.write_all(&[1u8]);
                            }
                            Err(e) => {
                                log::error!("[PROXY] Motion failed: {}", e);
                                let _ = stream.write_all(&[0u8]);
                            }
                        }
                        return true;
                    }
                }
                other => log::warn!("[PROXY] Unknown RPC method: {}", other),
            }
        }
        _ => log::warn!("[PROXY] Received unknown payload type"),
    }
    false
}

/// Convert a high-level motion request into encoder tick targets.
fn motion_ticks(req: &homerobot::MotionRequest) -> (i32, i32) {
    let (ticks_per_meter, wheel_base) = {
        let sizes = crate::constants::ROBOT_SIZES.lock().unwrap();
        (sizes.ticks_per_meter, sizes.wheel_base)
    };
    match req.r#type {
        0 => { // STRAIGHT
            let ticks = (req.distance * ticks_per_meter) as i32;
            (ticks, ticks)
        }
        1 => { // ROTATE
            let angle_rad = req.angle.to_radians();
            let dist = angle_rad * (wheel_base / 2.0);
            let ticks = (dist * ticks_per_meter) as i32;
            (-ticks, ticks)
        }
        2 => { // ARC
            let angle_rad = req.angle.to_radians();
            let left_dist = angle_rad * (req.radius - wheel_base / 2.0);
            let right_dist = angle_rad * (req.radius + wheel_base / 2.0);
            ((left_dist * ticks_per_meter) as i32, (right_dist * ticks_per_meter) as i32)
        }
        _ => (req.left_ticks, req.right_ticks),
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
    let bus = CommandBus::new();
    let world = Arc::new(Mutex::new(crate::world::WorldModel::new()));
    let sig_count = Arc::new(AtomicUsize::new(0));

    // 1. Initialize GTK GUI (only if not headless)
    let (gui_app, gui_tx) = if !is_headless {
        let (app, tx) = init_gui(bus.clone(), rec.clone());
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
    let bus_server = bus.clone();
    let world_server = world.clone();
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
                    let bus = bus_server.clone();
                    let w = world_server.clone();
                    let st = Arc::clone(&stats_server);
                    let sc = Arc::clone(&sig_count_server);
                    let gtx = gui_tx_server.clone();
                    let rrec = rec_server.clone();
                    thread::spawn(move || run_session(stream, bus, w, st, sc, gtx, rrec));
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                    sleep(Duration::from_millis(100));
                }
                _ => {}
            }
        }
    });

    // 4. Start Proxy Listener for cmd_sender
    let bus_proxy = bus.clone();
    let stats_proxy = stats.clone();
    thread::spawn(move || {
        let addr = "0.0.0.0:12346";
        let listener = TcpListener::bind(addr).expect("Could not bind Proxy");
        listener.set_nonblocking(true).ok();
        log::info!("[PROXY] Listening for cmd_sender on {}...", addr);

        while stats_proxy.running.load(Ordering::Relaxed) {
            match listener.accept() {
                Ok((stream, _)) => {
                    let bus = bus_proxy.clone();
                    thread::spawn(move || handle_proxy_client(stream, bus));
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
