use crate::homerobot::{server_to_robot_message, MotorMoveCommand};

#[derive(Debug, Clone, PartialEq)]
pub enum RobotCommand {
    StopAll,
    StopMoving,
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
        lidar_frequency: f32,
    },
    RunDiagnostic,
    SaveMap,
    AutonomousExploration { enabled: bool },
    NavigateTo { x: f32, y: f32 },
    ExecuteMotion {
        motion_type: i32,
        left_ticks: i32,
        right_ticks: i32,
        max_power: u32,
    },
    Reset,
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

impl RobotCommand {
    pub fn into_payload(&self) -> Option<server_to_robot_message::Payload> {
        match self {
            RobotCommand::ExecuteMotion { motion_type, left_ticks, right_ticks, max_power } => {
                use std::time::{SystemTime, UNIX_EPOCH};
                use prost::Message;
                let call_id = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis() as u32;
                
                let req = crate::homerobot::MotionRequest {
                    r#type: *motion_type,
                    distance: 0.0,
                    angle: 0.0,
                    radius: 0.0,
                    max_power: *max_power,
                    left_ticks: *left_ticks,
                    right_ticks: *right_ticks,
                };
                let mut payload_buf = Vec::new();
                req.encode(&mut payload_buf).unwrap();
                
                Some(server_to_robot_message::Payload::RpcRequest(crate::homerobot::RpcRequest {
                    call_id,
                    method: "ExecuteMotion".to_string(),
                    payload: payload_buf,
                }))
            }
            RobotCommand::RunDiagnostic => {
                use std::time::{SystemTime, UNIX_EPOCH};
                let call_id = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis() as u32;
                Some(server_to_robot_message::Payload::RpcRequest(crate::homerobot::RpcRequest {
                    call_id,
                    method: "RunDiagnostic".to_string(),
                    payload: vec![],
                }))
            }
            RobotCommand::StopAll => {
                Some(server_to_robot_message::Payload::StopAll(true))
            }
            RobotCommand::StopMoving => {
                Some(server_to_robot_message::Payload::MotorMove(MotorMoveCommand {
                    left_power: 0,
                    left_angle: 0.0,
                    right_power: 0,
                    right_angle: 0.0,
                }))
            }
            RobotCommand::MotorAngle { left_power, left_angle, right_power, right_angle } => {
                Some(server_to_robot_message::Payload::MotorMove(MotorMoveCommand {
                    left_power: *left_power as u32,
                    left_angle: *left_angle,
                    right_power: *right_power as u32,
                    right_angle: *right_angle,
                }))
            }
            RobotCommand::LidarControl { active, target_frequency_hz } => {
                Some(server_to_robot_message::Payload::LidarControl(crate::homerobot::LidarControlCommand {
                    active: *active,
                    target_frequency_hz: *target_frequency_hz,
                }))
            }
            RobotCommand::UpdateConfig { left_kp, left_ki, left_kd, right_kp, right_ki, right_kd, lidar_frequency } => {
                Some(server_to_robot_message::Payload::MotorConfig(crate::homerobot::RobotConfig {
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
                    lidar_frequency: *lidar_frequency,
                }))
            }
            RobotCommand::SaveMap => None,
            RobotCommand::AutonomousExploration { .. } => None,
            RobotCommand::NavigateTo { .. } => None,
            RobotCommand::Reset => None,
        }
    }
}
