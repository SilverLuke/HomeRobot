mod reader;
mod sensors;

use byteorder::{NetworkEndian, ReadBytesExt};
use std::f32::consts::PI;
use std::net::{TcpListener, TcpStream};
use std::thread::sleep;
use std::time::{Duration, Instant};
use std::{f32, io, thread};
use std::fmt::Formatter;
use crate::SendPacketType::TxMotorMove;

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

const PREFIX_SIZE: usize = 7;
const BUFFER_SIZE: usize = 16384;


#[allow(unused_variables)]
fn parse_imu_packet(size: u16, data: &Vec<u8>) {
    unimplemented!("TODO")
}

#[allow(unused_variables)]
fn parse_battery_packet(size: u16, data: &Vec<u8>) {
    unimplemented!("TODO")
}

#[allow(unused_variables)]
fn parse_encoder_packet(size: u16, data: &Vec<u8>) {
    unimplemented!("TODO")
}

#[allow(unused_variables)]
fn parse_config_packet(size: u16, data: &Vec<u8>) {
    unimplemented!("TODO")
}

#[allow(unused_variables)]
fn parse_echo_packet(size: u16, data: &Vec<u8>) {
    unimplemented!("TODO")
}


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

fn craft_motor_packet(right_power: i8, right_angle: f32, left_power:i8, left_angle: f32, millis:u32) -> Vec<u8> {
    let header = craft_header(millis, TxMotorMove, 10);

    // 17 is the length of the entire packet header 7 + 10 of the body
    let mut motor_packet = vec![0u8; 10];

    // Actual data
    // Right motor
    motor_packet[0] = right_power as u8;
    motor_packet[1..5].copy_from_slice(&(right_angle.to_be_bytes()));
    // Left motor
    motor_packet[5] = left_power as u8;
    motor_packet[6..10].copy_from_slice(&(left_angle.to_be_bytes()));

    [header, motor_packet].concat()
}




// Function to handle a connection
fn handle_connection(stream: TcpStream) {
    println!();
    println!("New connection from {}", stream.peer_addr().unwrap());
    stream.set_nonblocking(true).expect("set_nonblocking call failed");

    let mut protocol = reader::ProtocolManager::new(stream);

    let start_time = Instant::now();
    loop {
        let message = protocol.read_message();
        if message.is_err() {
            eprintln!("Error reading message: {:?}", message.err());
            eprintln!("Closing connection");
            return;
        } 
        else {
            message.unwrap().and_then(|msg| {
                println!("Received {}", &msg); 
                None::<HomeRobotPacket>
            });
        }

        //let millis = start_time.elapsed().as_millis() as u32;
        // loc_reader.send_message(millis);
        //sleep(Duration::from_millis(250));
    }
}


// Function to start the server
fn start_server(address: &str) -> io::Result<()> {
    let listener = TcpListener::bind(address)?;
    println!("Server listening on {}", address);

    for stream in listener.incoming() {
        match stream {
            Ok(stream) => {
                // Spawn a thread to handle each connection
                thread::spawn(move || handle_connection(stream));
            }
            Err(e) => eprintln!("Connection failed: {}", e),
        }
    }

    Ok(())
}

// Main function
fn main() -> io::Result<()> {
    // Start the server on all interfaces at port 12345
    start_server("0.0.0.0:12345")
}
