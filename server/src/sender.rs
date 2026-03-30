use std::net::TcpStream;
use std::sync::{Arc, Mutex};
use std::time::Instant;
use crate::homerobot::ServerToRobotMessage;
use crate::reader::ProtocolManager;
use crate::stats::Stats;
use crate::command::RobotCommand;
use prost::Message;

pub fn send_manual_command(
    robot_command: Arc<Mutex<RobotCommand>>,
    protocol: &mut ProtocolManager<TcpStream>,
    start_time: Instant,
    last_sent_command: &mut RobotCommand,
    stats: Arc<Stats>)
{
    let mut new_command = None;
    if let Ok(current_command) = robot_command.lock() {
        new_command = Some(current_command.clone());
    }

    if let Some(current_command) = new_command {
        if current_command != *last_sent_command {
            let millis = start_time.elapsed().as_millis() as u32;

            let msg = ServerToRobotMessage {
                sequence_millis: millis,
                payload: current_command.into_payload(),
            };

            let mut buf = Vec::new();
            msg.encode(&mut buf).unwrap();

            // Add 2-byte length prefix
            let len = buf.len() as u16;
            let mut final_packet = len.to_be_bytes().to_vec();
            final_packet.extend(buf);

            if let Err(e) = protocol.send_packet(&final_packet) {
                eprintln!("Error sending motor command: {:?}\r", e);
            } else {
                stats.log(&format!("[CMD] Sent: {:?}", current_command));
            }

            *last_sent_command = current_command;
        }
    }
}
