mod reader;
mod sensors;

use crate::SendPacketType::TxMotorMove;
use byteorder::{NetworkEndian, WriteBytesExt};
use std::fmt::Formatter;
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::thread::sleep;
use std::time::{Duration, Instant};
use std::{f32, io, thread};

// for reading data in Little Endian
use num_enum::TryFromPrimitive;

// Enum for Sensors
#[repr(u8)]
#[derive(Debug, Copy, Clone, TryFromPrimitive, PartialEq)]
enum ReceivePacketType {
    RxLidar = 0,
    RxImu = 1,
    RxBattery = 2,
    RxEncoderMotor = 4,
    RxConfig = 8,
    RxEcho = 16
}

// Enum for ActionTypes
#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq)]
enum SendPacketType {
    TxMotorMove = 0,
    TxMotorConfig = 1,
    TxLidarMotor = 2,
    TxStopAll = 4,
    TxRequest = 8
}

// Enum for PacketType which is a union of Sensors and ActionType
#[derive(Debug, Copy, Clone)]
#[repr(u8)]
#[derive(PartialEq)]
enum PacketType {
    Send(SendPacketType),
    Receive(ReceivePacketType),
}

// Motor command structure
#[derive(Debug, Clone, Copy, PartialEq)]
struct MotorCommand {
    right_power: u8,
    right_angle: f32,
    left_power: u8,
    left_angle: f32,
}

impl Default for MotorCommand {
    fn default() -> Self {
        Self {
            right_power: 0,
            right_angle: 0.0,
            left_power: 0,
            left_angle: 0.0,
        }
    }
}

/**
 * @struct HomeRobotPacket
 * @brief This struct is used to send data from the ESP32 to the PC
 * @var HomeRobotPacket::sequence_millis: when the packet was created, used to
 * sync the data between the ESP32 and the PC
 * @var HomeRobotPacket::type: the type of the packet, used to identify the data
 * @var HomeRobotPacket::size: the size of the data in bytes
 * @var HomeRobotPacket::data: the data itself
 */
struct HomeRobotPacket {
    sequence_millis:u32,
    packet_type: PacketType,
    size: u16,
    data: Box<Vec<u8>>
}

impl std::fmt::Display for HomeRobotPacket {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "HomeRobotPacket {{ sequence_millis: {}, packet_type: {:?}, size: {}, data: {:?} }}",
            self.sequence_millis,
            self.packet_type,
            self.size,
            self.data.len()
        )
    }
}

const PREFIX_SIZE: usize = 7;
const BUFFER_SIZE: usize = 16384;

fn craft_header(millis:u32, packet_type: SendPacketType, length: u16) -> Vec<u8> {
    let mut header = vec![0u8; 7];
    // In the firsts 4 bytes put millis
    header[0..4].copy_from_slice(&(millis.to_be_bytes()));

    // Then the type
    header[4] = packet_type as u8;

    // 2 bytes for the length
    header[5..7].copy_from_slice(&(length.to_be_bytes()));
    header
}


fn craft_motor_packet(right_power: u8, right_angle: f32, left_power:u8, left_angle: f32, millis:u32) -> Vec<u8> {
    let header = craft_header(millis, TxMotorMove, 10);

    // 17 is the length of the entire packet header 7 + 10 of the body
    let mut motor_packet = vec![0u8; 10];

    // Actual data
    // Right motor
    motor_packet[0] = right_power;
    (&mut motor_packet[1..5]).write_f32::<NetworkEndian>(right_angle).unwrap();
    // Left motor
    motor_packet[5] = left_power;
    (&mut motor_packet[6..10]).write_f32::<NetworkEndian>(left_angle).unwrap();

    [header, motor_packet].concat()
}

use crossterm::{
    event,
    event::{Event, KeyCode, KeyEvent},
    terminal::{disable_raw_mode, enable_raw_mode},
};

const POWER: u8 = 127;

fn handle_keyboard_input(motor_command: Arc<Mutex<MotorCommand>>) {
    println!("Keyboard control started. Use WASD keys to control the robot:");
    println!("W - Forward (both motors)");
    println!("A - Turn left (left motor only)");
    println!("D - Turn right (right motor only)");
    println!("S - Backward (both motors)");
    println!("Space - Stop");
    println!("'q' - Quit");
    println!("Keys will be detected immediately without pressing Enter!");

    // Enable raw mode for immediate key detection
    if let Err(e) = enable_raw_mode() {
        eprintln!("Failed to enable raw mode: {}", e);
        return;
    }

    loop {
        // Check if there's an event available
        if event::poll(Duration::from_millis(100)).unwrap_or(false) {
            match event::read() {
                Ok(Event::Key(KeyEvent { code, .. })) => {
                    let new_command = match code {
                        KeyCode::Char('w') | KeyCode::Char('W') => {
                            MotorCommand {
                                right_power: POWER,
                                right_angle: 1.0,
                                left_power: POWER,
                                left_angle: 1.0,
                            }
                        },
                        KeyCode::Char('s') | KeyCode::Char('S') => {
                            MotorCommand {
                                right_power: POWER,
                                right_angle: -1.0,
                                left_power: POWER,
                                left_angle: -1.0,
                            }
                        },
                        KeyCode::Char('a') | KeyCode::Char('A') => {
                            MotorCommand {
                                right_power: 0,
                                right_angle: 0.0,
                                left_power: POWER,
                                left_angle: 1.0,
                            }
                        },
                        KeyCode::Char('d') | KeyCode::Char('D') => {
                            MotorCommand {
                                right_power: POWER,
                                right_angle: 1.0,
                                left_power: 0,
                                left_angle: 0.0,
                            }
                        },
                        KeyCode::Char(' ') => {
                            println!("Stopping");
                            MotorCommand::default()
                        },
                        KeyCode::Char('q') | KeyCode::Char('Q') | KeyCode::Esc => {
                            println!("Exiting keyboard control...");
                            break;
                        },
                        _ => {
                            // For any other key, don't change the command
                            continue;
                        }
                    };

                    // Update the shared motor command
                    if let Ok(mut cmd) = motor_command.lock() {
                        *cmd = new_command;
                    }
                },
                Ok(_) => {
                    // Other events (mouse, resize, etc.) - ignore
                },
                Err(e) => {
                    eprintln!("Error reading input: {}", e);
                    break;
                }
            }
        }
    }

    // Restore normal terminal mode
    if let Err(e) = disable_raw_mode() {
        eprintln!("Failed to disable raw mode: {}", e);
    }
}

// Function to handle a connection
fn handle_connection(stream: TcpStream, motor_command: Arc<Mutex<MotorCommand>>) {
    println!("New connection from {}", stream.peer_addr().unwrap());
    stream.set_nonblocking(true).expect("set_nonblocking call failed");

    let mut protocol = reader::ProtocolManager::new(stream);
    let start_time = Instant::now();
    let mut last_command_time = Instant::now();
    let command_interval = Duration::from_millis(100); // Send commands every 100ms
    let mut last_sent_command = MotorCommand::default(); // Track the last sent command
    
    loop {
        // Check for incoming messages from robot
        let message = protocol.read_message();
        if message.is_err() {
            eprintln!("Error reading message: {:?}", message.err());
            eprintln!("Closing connection");
            return;
        } else {
            message.unwrap().and_then(|msg| {
                println!("Received {}", &msg); 
                None::<HomeRobotPacket>
            });
        }

        // Send motor commands periodically, but only if they've changed
        if last_command_time.elapsed() >= command_interval {
            let mut new_command = None;
            if let Ok(current_command) = motor_command.lock() {
                new_command = Some(*current_command);
            }
            if let Some(current_command) = new_command {
                // Only send it if the command is different from the last one sent
                if current_command != last_sent_command {
                    let millis = start_time.elapsed().as_millis() as u32;
                    let packet = craft_motor_packet(
                        current_command.right_power,
                        current_command.right_angle,
                        current_command.left_power,
                        current_command.left_angle,
                        millis
                    );
                    
                    println!("Sending motor command: Left power: {}, Right power: {}", 
                            current_command.left_power as f32 * current_command.left_angle, current_command.right_power as f32 * current_command.right_angle);

                    if let Err(e) = protocol.send_packet(&packet) {
                        eprintln!("Error sending motor command: {:?}", e);
                        return;
                    }
                    
                    // Update the last sent command
                    last_sent_command = current_command;
                } 
                
                last_command_time = Instant::now();
            }
        }

        sleep(Duration::from_millis(10)); // Small delay to prevent busy waiting
    }
}

// Function to start the server
fn start_server(address: &str) -> io::Result<()> {
    let listener = TcpListener::bind(address)?;
    println!("Server listening on {}", address);

    // Shared motor command state
    let motor_command = Arc::new(Mutex::new(MotorCommand::default()));

    // Start keyboard input handler in a separate thread
    let motor_command_clone = Arc::clone(&motor_command);
    thread::spawn(move || {
        handle_keyboard_input(motor_command_clone);
    });

    let mut connection_count = 0;

    for stream in listener.incoming() {
        match stream {
            Ok(stream) => {
                connection_count += 1;
                println!("Connection #{} established from {}", 
                        connection_count, 
                        stream.peer_addr().unwrap_or_else(|_| "unknown".parse().unwrap()));
                
                // Clone the motor command reference for this connection
                let motor_command_clone = Arc::clone(&motor_command);
                
                // Spawn a thread to handle each connection
                thread::spawn(move || handle_connection(stream, motor_command_clone));
                
                // Continue listening for more connections (no break statement)
            }
            Err(e) => eprintln!("Connection failed: {}", e),
        }
    }

    Ok(())
}

// Main function
fn main() -> io::Result<()> {
    println!("Robot Control Server");
    println!("====================");
    
    // Start the server on all interfaces at port 12345
    start_server("0.0.0.0:12345")
}