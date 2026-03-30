use crate::homerobot::{server_to_robot_message, MotorMoveCommand};

#[derive(Debug, Clone, PartialEq)]
pub enum RobotCommand {
    StopAll,
    StopMoving,
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
    RunDiagnostic,
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
            RobotCommand::MotorDirect { left_speed, right_speed } => {
                Some(server_to_robot_message::Payload::MotorMove(MotorMoveCommand {
                    left_power: left_speed.abs() as u32,
                    left_angle: if *left_speed >= 0 { 1.0 } else { -1.0 },
                    right_power: right_speed.abs() as u32,
                    right_angle: if *right_speed >= 0 { 1.0 } else { -1.0 },
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
            RobotCommand::UpdateConfig { left_kp, left_ki, left_kd, right_kp, right_ki, right_kd } => {
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
                    lidar_frequency: 5.0,
                }))
            }
            RobotCommand::RequestData => {
                Some(server_to_robot_message::Payload::RequestData(true))
            }
        }
    }
}
