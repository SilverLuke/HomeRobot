mod reader;
mod sensors;
mod input;
mod sender;
mod constants;

use byteorder::WriteBytesExt;
use std::fmt::Formatter;
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::thread::sleep;
use std::time::{Duration, Instant};
use std::{io, thread};

use crate::input::{handle_input, print_joystick_info, print_summary};
use crate::sender::{send_manual_command, MotorCommand};
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

// Function to handle a connection
fn handle_connection(stream: TcpStream, motor_command: Arc<Mutex<MotorCommand>>) {
    println!("New connection from {}", stream.peer_addr().unwrap());
    stream.set_nonblocking(true).expect("set_nonblocking call failed");

    let mut protocol = reader::ProtocolManager::new(stream);
    let start_time = Instant::now();
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

        send_manual_command(motor_command.clone(), &mut protocol, start_time, &mut last_sent_command);

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
        let sdl_context = sdl2::init().unwrap();
        print_summary();
        print_joystick_info(&sdl_context.joystick().unwrap());
        handle_input(motor_command_clone, &sdl_context);
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