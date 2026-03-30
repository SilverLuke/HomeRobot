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
use crate::sender::send_manual_command;
use crate::homerobot::robot_to_server_message::Payload;

/// Handles a single robot connection
pub fn handle_connection(stream: TcpStream, robot_command: Arc<Mutex<RobotCommand>>, stats: Arc<Stats>, sig_count: Arc<AtomicUsize>, gui_tx: mpsc::Sender<GuiUpdate>) {
    stats.active_connections.fetch_add(1, Ordering::SeqCst);
    let addr = stream.peer_addr().unwrap_or_else(|_| "unknown".parse().unwrap());
    stats.log(&format!("[CONN] New connection from {}", addr));
    let _ = gui_tx.send(GuiUpdate::Status(format!("Connected: {}", addr)));
    
    stream.set_nonblocking(true).ok();
    stream.set_read_timeout(Some(Duration::from_secs(5))).ok();

    let mut protocol = reader::ProtocolManager::new(stream, stats.clone());
    let start_time = Instant::now();
    let mut last_sent_command = RobotCommand::default();
    
    while stats.running.load(Ordering::Relaxed) && sig_count.load(Ordering::Relaxed) == 0 {
        loop {
            match protocol.read_message() {
                Ok(Some(msg)) => {
                    if let Some(payload) = msg.payload {
                        match payload {
                            Payload::Battery(bat) => {
                                let _ = gui_tx.send(GuiUpdate::Battery { percentage: bat.percentage, voltage_mv: bat.voltage_mv });
                            }
                            Payload::Encoders(enc) => {
                                let _ = gui_tx.send(GuiUpdate::Encoders { left: enc.left_encoder, right: enc.right_encoder });
                            }
                            Payload::Imu(imu) => {
                                if let (Some(a), Some(g)) = (imu.acceleration, imu.gyroscope) {
                                    let _ = gui_tx.send(GuiUpdate::Imu { 
                                        ax: a.x, ay: a.y, az: a.z,
                                        gx: g.x, gy: g.y, gz: g.z
                                    });
                                }
                            }
                            Payload::Lidar(scan) => {
                                let _ = gui_tx.send(GuiUpdate::Lidar(scan.points));
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
        sleep(Duration::from_millis(10));
    }

    stats.active_connections.fetch_sub(1, Ordering::SeqCst);
    stats.log(&format!("[CONN] Closing connection to {}", addr));
    let _ = gui_tx.send(GuiUpdate::Status("Idle".to_string()));
}
