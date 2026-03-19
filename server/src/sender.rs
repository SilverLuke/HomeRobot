use std::net::TcpStream;
use std::sync::{Arc, Mutex};
use std::time::Instant;
use crate::homerobot::{ServerToRobotMessage, MotorMoveCommand, server_to_robot_message};
use crate::reader::ProtocolManager;
use prost::Message;

// Robot command structure representing all possible commands to the robot
#[derive(Debug, Clone, PartialEq)]
pub enum RobotCommand {
    StopAll,
    MotorDirect { 
        left_speed: i16,
        right_speed: i16,
    },
    MotorAngle { 
        left_power: u8, 
        left_angle: f32,
        right_power: u8,
        right_angle: f32,
    },
    LidarControl {
        active: bool,
        target_frequency_hz: f32,
    },
    UpdateConfig {
        left_kp: f32,
        left_ki: f32,
        left_kd: f32,
        right_kp: f32,
        right_ki: f32,
        right_kd: f32,
    },
    RequestData,
}

impl Default for RobotCommand {
    fn default() -> Self {
        Self::MotorAngle {
            left_power: 0,
            left_angle: 0.0,
            right_power: 0,
            right_angle: 0.0,
        }
    }
}

pub fn send_manual_command(
    robot_command: Arc<Mutex<RobotCommand>>,
    protocol: &mut ProtocolManager<TcpStream>,
    start_time: Instant,
    last_sent_command: &mut RobotCommand)
{
    let mut new_command = None;
    if let Ok(current_command) = robot_command.lock() {
        new_command = Some(current_command.clone());
    }

    if let Some(current_command) = new_command {
        if current_command != *last_sent_command {
            let millis = start_time.elapsed().as_millis() as u32;

            let mut msg = ServerToRobotMessage {
                sequence_millis: millis,
                payload: None,
            };

            match &current_command {
                RobotCommand::StopAll => {
                    msg.payload = Some(server_to_robot_message::Payload::StopAll(true));
                }
                RobotCommand::MotorDirect { left_speed, right_speed } => {
                    msg.payload = Some(server_to_robot_message::Payload::MotorMove(MotorMoveCommand {
                        left_power: left_speed.abs() as u32,
                        left_angle: if *left_speed >= 0 { 1.0 } else { -1.0 },
                        right_power: right_speed.abs() as u32,
                        right_angle: if *right_speed >= 0 { 1.0 } else { -1.0 },
                    }));
                }
                RobotCommand::MotorAngle { left_power, left_angle, right_power, right_angle } => {
                    msg.payload = Some(server_to_robot_message::Payload::MotorMove(MotorMoveCommand {
                        left_power: *left_power as u32,
                        left_angle: *left_angle,
                        right_power: *right_power as u32,
                        right_angle: *right_angle,
                    }));
                }
                RobotCommand::LidarControl { active, target_frequency_hz } => {
                    msg.payload = Some(server_to_robot_message::Payload::LidarControl(crate::homerobot::LidarControlCommand {
                        active: *active,
                        target_frequency_hz: *target_frequency_hz,
                    }));
                }
                RobotCommand::UpdateConfig { left_kp, left_ki, left_kd, right_kp, right_ki, right_kd } => {
                    msg.payload = Some(server_to_robot_message::Payload::MotorConfig(crate::homerobot::RobotConfig {
                        left_motor: Some(crate::homerobot::MotorPidConfig {
                            kp: *left_kp,
                            ki: *left_ki,
                            kd: *left_kd,
                            max_speed: 255,
                        }),
                        right_motor: Some(crate::homerobot::MotorPidConfig {
                            kp: *right_kp,
                            ki: *right_ki,
                            kd: *right_kd,
                            max_speed: 255,
                        }),
                        lidar_frequency: 5.0,
                    }));
                }
                RobotCommand::RequestData => {
                    msg.payload = Some(server_to_robot_message::Payload::RequestData(true));
                }
            }

            let mut buf = Vec::new();
            msg.encode(&mut buf).unwrap();

            // Add 2-byte length prefix
            let len = buf.len() as u16;
            let mut final_packet = len.to_be_bytes().to_vec();
            final_packet.extend(buf);

            if let Err(e) = protocol.send_packet(&final_packet) {
                eprintln!("Error sending motor command: {:?}\r", e);
            }

            *last_sent_command = current_command;
        }
    }
}
