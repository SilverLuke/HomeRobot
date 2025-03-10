use std::f32::consts::PI;
use std::io::{self, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::{f32, thread};
use std::path::Prefix;
use std::thread::sleep;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use byteorder::{BigEndian, LittleEndian, NetworkEndian, ReadBytesExt}; // for reading data in Little Endian
use num_enum::{TryFromPrimitive};
use crate::SendPacketType::TxMotorMove;

// Enum for Sensors
#[repr(u8)]
#[derive(Debug, TryFromPrimitive)]
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
#[derive(Debug)]
enum SendPacketType {
    TxMotorMove = 0,
    TxMotorConfig = 1,
    TxLidarMotor = 2,
    TxStopAll = 4,
    TxRequest = 8
}

// Enum for PacketType which is a union of Sensors and ActionType
#[derive(Debug)]
#[repr(u8)]
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
    data: Vec<u8>
}

const PREFIX_SIZE: usize = 7;

// Function to handle individual packets from the stream
fn receive_header(mut stream: &TcpStream) -> io::Result<HomeRobotPacket> {
    println!("Receive header");

    // Buffer to read the packet's header (4 bytes for millis, 1 byte for type, 2 bytes for size)
    let mut header = [0u8; PREFIX_SIZE];
    stream.read_exact(&mut header)?;
    println!("Read first {PREFIX_SIZE} bytes");
    // Read millis (u32, 4 bytes)
    let millis = (&header[0..4]).read_u32::<LittleEndian>()?;
    println!("Millis: {}", millis);

    // Read sensor type (u8, 1 byte)
    let packet_type_value_raw = header[4];
    let packet_type = ReceivePacketType::try_from(packet_type_value_raw).unwrap();
    println!("Packet type {packet_type:?}");

    // Read the size of the data (u16, 2 bytes)
    let size = (&header[5..7]).read_u16::<LittleEndian>()?;
    println!("Data size: {size}");

    // Read the actual data based on size
    let mut data = vec![0u8; size as usize];
    stream.read_exact(&mut data)?;


    let packet = HomeRobotPacket {
        sequence_millis: millis,
        packet_type: PacketType::Receive(packet_type),
        size,
        data,
    };

    Ok(packet)
}

fn parse_data(packet: &HomeRobotPacket) {
    // Extract fields directly from the byte vector
    match &packet.packet_type {
        PacketType::Send(_) => {}
        PacketType::Receive(rx_type) => {
            match rx_type {
                ReceivePacketType::RxLidar => {
                    parse_lidar_packet(packet.size, &packet.data);
                }
                ReceivePacketType::RxImu => {
                    parse_imu_packet(packet.size, &packet.data);
                }
                ReceivePacketType::RxBattery => {
                    parse_battery_packet(packet.size, &packet.data);
                }
                ReceivePacketType::RxEncoderMotor => {
                    parse_encoder_packet(packet.size, &packet.data);
                }
                ReceivePacketType::RxConfig => {
                    parse_config_packet(packet.size, &packet.data);
                }
                ReceivePacketType::RxEcho => {
                    parse_echo_packet(packet.size, &packet.data);
                }
            }
        }
    }
}

fn parse_lidar_packet(size: u16, data: &Vec<u8>) {
    // Size of a single lidar packet in bytes
    const SINGLE_PACKET_SIZE: u16 = 5;
    assert_eq!(size as usize, data.len(), "Vector data and size must be equal");
    assert_eq!(size % SINGLE_PACKET_SIZE, 0, "Check if all packets are complete");
    let packets = size / SINGLE_PACKET_SIZE;

    for i in 0..packets {
        let start: usize = (i as usize) * SINGLE_PACKET_SIZE as usize;
        let end: usize = (i + 1) as usize * SINGLE_PACKET_SIZE as usize;
        parse_single(data[start..end].to_owned());
    }

    fn parse_single(data: Vec<u8>) {
        let sync_quality = data[0];
        let angle_q6_check_bit = (&data[1..3]).read_u16::<NetworkEndian>().unwrap();
        let distance_q2 = (&data[3..5]).read_u16::<NetworkEndian>().unwrap();

        // Perform the operations
        let scan_completed = (sync_quality & (0x1<<0)) != 0; // Extract syncbit
        let distance_mm = (distance_q2 as f32) * 0.25;
        let angle_deg = ((angle_q6_check_bit >> 1) as f32) * 0.015625; // Shift and scale
        let quality = (sync_quality >> 2) & 0x3F; // Extract the 6-bit quality

        // Calculate position: x, y based on angle and distance
        let angle_rad = angle_deg * PI / 180.0;
        let distance_meters = distance_mm / 1000.0;
        let x_pos = distance_meters * f32::cos(angle_rad);
        let y_pos = distance_meters * f32::sin(angle_rad);

        // Print the results
        println!("Packet Type: Lidar Completed: {scan_completed}, \
Distance (mm): {distance_mm:.2} Angle (deg): {angle_deg:.2} Quality {quality}");
        //println!("Vector <{}, {}>", x_pos, y_pos);
    }
}

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

fn read_message(stream: &TcpStream) {
    match receive_header(stream) {
        Ok(header_data) => {
            parse_data(&header_data);
        }
        Err(e) => eprintln!("Error processing packet: {}", e),
    }
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

fn send_message(stream: &mut TcpStream, millis: u32) {
    let motor_packet =
    // Every second change the robot motion
    if (millis / 5000) % 2 == 0 {
        // Go forward
        println!("Sending motor packet, forward. {millis}");
        craft_motor_packet(50, f32::INFINITY, 50, f32::INFINITY, millis)
    } else {
        // Go backward
        println!("Sending motor packet, backward. {millis}");
        craft_motor_packet(-50, f32::INFINITY, -50, f32::INFINITY, millis)
    };

    stream.write_all(&motor_packet).unwrap()
}

// Function to handle a connection
fn handle_connection(mut stream: TcpStream) {
    println!("New connection from {}", stream.peer_addr().unwrap());
    let start_time = Instant::now();
    loop {
//      read_message(&stream);

        let millis = start_time.elapsed().as_millis() as u32;
        send_message(&mut stream, millis);
        sleep(Duration::from_millis(250));
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
