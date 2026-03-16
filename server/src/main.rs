mod reader;
mod input;
mod sender;
mod constants;

pub mod homerobot {
    include!(concat!(env!("OUT_DIR"), "/homerobot.rs"));
}

use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::thread::sleep;
use std::time::{Duration, Instant};
use std::{io, thread};

use crate::input::{handle_input, print_joystick_info, print_summary};
use crate::sender::{send_manual_command, RobotCommand};
use crate::homerobot::robot_to_server_message::Payload;

// Function to handle a connection
fn handle_connection(stream: TcpStream, robot_command: Arc<Mutex<RobotCommand>>) {
    println!("New connection from {}", stream.peer_addr().unwrap());
    stream.set_nonblocking(true).expect("set_nonblocking call failed");

    let mut protocol = reader::ProtocolManager::new(stream);
    let start_time = Instant::now();
    let mut last_sent_command = RobotCommand::default();
    
    loop {
        // Check for incoming messages from robot
        match protocol.read_message() {
            Ok(Some(msg)) => {
                if let Some(payload) = msg.payload {
                    match payload {
                        Payload::Imu(imu) => {
                            println!("[IMU] Accel: {:?}, Gyro: {:?}", imu.acceleration, imu.gyroscope);
                        }
                        Payload::Battery(bat) => {
                            println!("[BATTERY] {}%, {} mV", bat.percentage, bat.voltage_mv);
                        }
                        Payload::Lidar(scan) => {
                            if !scan.points.is_empty() {
                                println!("[LIDAR] Received {} points", scan.points.len());
                                for (i, point) in scan.points.iter().enumerate().take(5) {
                                    println!("  [{}] Dist: {:.2}mm, Angle: {:.2}deg, Quality: {}", 
                                            i, point.distance_mm, point.angle_deg, point.quality);
                                }
                                if scan.points.len() > 5 {
                                    println!("  ...");
                                }
                            } else {
                                println!("[LIDAR] Received empty scan data");
                            }
                        }
                        Payload::Heartbeat(_) => {
                            // Heartbeat is useful to keep the connection alive
                            // and know the robot is still there.
                            // println!("[HEARTBEAT] Received");
                        }
                        Payload::Encoders(enc) => {
                            println!("[ENCODERS] L: {}, R: {}", enc.left_encoder, enc.right_encoder);
                        }
                        Payload::Config(config) => {
                            println!("[CONFIG] Received from robot:");
                            if let Some(left) = config.left_motor {
                                println!("  Left Motor: Kp={:.2}, Ki={:.2}, Kd={:.2}, MaxSpeed={}", 
                                        left.kp, left.ki, left.kd, left.max_speed);
                            }
                            if let Some(right) = config.right_motor {
                                println!("  Right Motor: Kp={:.2}, Ki={:.2}, Kd={:.2}, MaxSpeed={}", 
                                        right.kp, right.ki, right.kd, right.max_speed);
                            }
                            println!("  Lidar Frequency: {:.2} Hz", config.lidar_frequency);
                        }
                    }
                }
            }
            Ok(None) => {}
            Err(e) => {
                eprintln!("Error reading message: {:?}", e);
                eprintln!("Closing connection");
                return;
            }
        }

        send_manual_command(robot_command.clone(), &mut protocol, start_time, &mut last_sent_command);

        sleep(Duration::from_millis(10));
    }
}

// Function to start the server
fn start_server(address: &str) -> io::Result<()> {
    let listener = TcpListener::bind(address)?;
    println!("Server listening on {}", address);

    let robot_command = Arc::new(Mutex::new(RobotCommand::StopAll));

    let robot_command_clone = Arc::clone(&robot_command);
    thread::spawn(move || {
        let sdl_context = sdl2::init().unwrap();
        print_summary();
        print_joystick_info(&sdl_context.joystick().unwrap());
        handle_input(robot_command_clone, &sdl_context);
    });

    let mut connection_count = 0;

    for stream in listener.incoming() {
        match stream {
            Ok(stream) => {
                connection_count += 1;
                println!("Connection #{} established from {}", 
                        connection_count, 
                        stream.peer_addr().unwrap_or_else(|_| "unknown".parse().unwrap()));
                
                let robot_command_clone = Arc::clone(&robot_command);
                thread::spawn(move || handle_connection(stream, robot_command_clone));
            }
            Err(e) => eprintln!("Connection failed: {}", e),
        }
    }

    Ok(())
}

fn main() -> io::Result<()> {
    println!("Robot Control Server (Protobuf)");
    println!("===============================");
    
    start_server("0.0.0.0:12345")
}
