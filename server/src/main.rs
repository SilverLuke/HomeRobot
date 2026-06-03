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
    println!("[PROXY] cmd_sender connected.");
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
                                        println!("[PROXY] Received RPC Request: {}", r.method);
                                        if r.method == "RunDiagnostic" {
                                            *cmd = RobotCommand::RunDiagnostic;
                                            println!("[PROXY] Forwarding RunDiagnostic");
                                        } else if r.method == "SaveMap" {
                                            *cmd = RobotCommand::SaveMap;
                                            println!("[PROXY] Triggering SaveMap");
                                        } else if r.method == "StartExplore" {
                                            *cmd = RobotCommand::AutonomousExploration { enabled: true };
                                            println!("[PROXY] Starting Autonomous Exploration");
                                        } else if r.method == "StopExplore" {
                                            *cmd = RobotCommand::AutonomousExploration { enabled: false };
                                            println!("[PROXY] Stopping Autonomous Exploration");
                                        }
                                    }
                                    _ => {
                                        println!("[PROXY] Received unknown payload type");
                                    }
                                }
                            }
                        } else {
                            println!("[PROXY] Received message with no payload");
                        }
                    } else {
                        println!("[PROXY] Failed to decode Protobuf message");
                    }
                }
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => { continue; }
            Err(_) => {
                println!("[PROXY] cmd_sender disconnected.");
                break;
            }
        }
    }
}

fn main() -> io::Result<()> {
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
    let robot_command = Arc::new(Mutex::new(RobotCommand::StopAll));
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
        println!("[SERVER] Listening on {}...\r", addr);

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
        println!("[PROXY] Listening for cmd_sender on {}...\r", addr);

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

    // 5. Run GTK GUI or Headless Loop
    if is_headless {
        println!("[SERVER] Running in background. Waiting for connections...");
        while stats.running.load(Ordering::Relaxed) {
            sleep(Duration::from_millis(100));
        }
    } else if let Some(app) = gui_app {
        app.run();
    }

    println!("\n\r[SHUTDOWN] Exiting...\r");
    Ok(())
}
