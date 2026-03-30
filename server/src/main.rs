mod reader;
mod input;
mod sender;
mod constants;
mod stats;

pub mod homerobot {
    include!(concat!(env!("OUT_DIR"), "/homerobot.rs"));
}

use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::thread::sleep;
use std::time::{Duration, Instant};
use std::{io, thread, process};
use std::io::Write;

use crate::input::{handle_input, print_joystick_info, print_summary};
use crate::sender::{send_manual_command, RobotCommand};
use crate::homerobot::robot_to_server_message::Payload;
use crate::stats::Stats;

/// Handles a single robot connection
fn handle_connection(stream: TcpStream, robot_command: Arc<Mutex<RobotCommand>>, stats: Arc<Stats>, sig_count: Arc<AtomicUsize>) {
    stats.active_connections.fetch_add(1, Ordering::SeqCst);
    let addr = stream.peer_addr().unwrap_or_else(|_| "unknown".parse().unwrap());
    stats.log(&format!("[CONN] New connection from {}", addr));
    
    stream.set_nonblocking(true).ok();
    stream.set_read_timeout(Some(Duration::from_secs(5))).ok();

    let mut protocol = reader::ProtocolManager::new(stream, stats.clone());
    let start_time = Instant::now();
    let mut last_sent_command = RobotCommand::default();
    
    while stats.running.load(Ordering::Relaxed) && sig_count.load(Ordering::Relaxed) == 0 {
        // Drain all available messages
        loop {
            match protocol.read_message() {
                Ok(Some(msg)) => {
                    if let Some(payload) = msg.payload {
                        match payload {
                            Payload::Battery(bat) => {
                                stats.log(&format!("[BATTERY] {}%, {} mV", bat.percentage, bat.voltage_mv));
                            }
                            Payload::Encoders(enc) => {
                                stats.log(&format!("[ENCODERS] L: {}, R: {}", enc.left_encoder, enc.right_encoder));
                            }
                            Payload::Config(_) => {
                                stats.log("[CONFIG] Robot configuration updated");
                            }
                            Payload::RpcResponse(resp) => {
                                stats.log(&format!("[RPC RESPONSE] ID: {}, Error: {}", resp.call_id, resp.error));
                                if let Ok(diag_result) = prost::Message::decode(&*resp.payload) {
                                    let diag_result: crate::homerobot::DiagnosticResult = diag_result;
                                    stats.log(&format!("[DIAGNOSTICS] All OK: {}", diag_result.all_ok));
                                    for check in diag_result.checks {
                                        stats.log(&format!("  - {}: {} ({})", check.name, if check.success { "PASS" } else { "FAIL" }, check.message));
                                    }
                                } else {
                                    stats.log("[RPC ERROR] Failed to decode DiagnosticResult from payload");
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
                    return; // Exit handle_connection
                }
            }
        }

        send_manual_command(robot_command.clone(), &mut protocol, start_time, &mut last_sent_command);
        sleep(Duration::from_millis(10));
    }

    stats.active_connections.fetch_sub(1, Ordering::SeqCst);
    stats.log(&format!("[CONN] Closing connection to {}", addr));
}

fn main() -> io::Result<()> {
    // 0. Disable SDL2 Signal Catching
    std::env::set_var("SDL_NO_SIGNAL_HANDLERS", "1");

    let stats = Stats::new();
    let robot_command = Arc::new(Mutex::new(RobotCommand::StopAll));
    let sig_count = Arc::new(AtomicUsize::new(0));

    // 1. Cross-platform Signal Handling
    let sc = sig_count.clone();
    let stats_signal = stats.clone();
    ctrlc::set_handler(move || {
        let count = sc.fetch_add(1, Ordering::SeqCst) + 1;
        if count == 1 {
            stats_signal.running.store(false, Ordering::SeqCst);
            // Print directly to stderr for immediate visibility
            let _ = writeln!(io::stderr(), "\r\n[CTRL+C] Shutdown initiated. Press again to force exit.\r");
        } else {
            let _ = writeln!(io::stderr(), "\r\n[CTRL+C] Force exit requested.\r");
            process::exit(0);
        }
    }).expect("Error setting Ctrl-C handler");

    println!("======================================\r");
    println!("      Robot Control Server v0.8       \r");
    println!("======================================\r");
    // 2. Start Listener
    let stats_server = stats.clone();
    let rc_server = robot_command.clone();
    let sig_count_server = sig_count.clone();
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
                    thread::spawn(move || handle_connection(stream, rc, st, sc));
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                    sleep(Duration::from_millis(100));
                }
                _ => {}
            }
        }
    });

    // 3. SDL & Terminal Input Loop (Main Thread)
    // We wrap this in enable_raw_mode to allow terminal input to be captured instantly.
    crossterm::terminal::enable_raw_mode().unwrap();
    
    if let Ok(sdl_context) = sdl2::init() {
        print_summary();
        if let Ok(joystick) = sdl_context.joystick() {
            let _ = print_joystick_info(&joystick);
        }
        
        handle_input(robot_command.clone(), &sdl_context, stats.clone(), sig_count.clone());
    } else {
        writeln!(io::stderr(), "[ERROR] Failed to initialize SDL2 input subsystem\r").unwrap();
        while stats.running.load(Ordering::Relaxed) && sig_count.load(Ordering::Relaxed) == 0 {
            sleep(Duration::from_millis(500));
        }
    }

    let _ = crossterm::terminal::disable_raw_mode();
    println!("\n\r[SHUTDOWN] Exiting...\r");
    Ok(())
}
