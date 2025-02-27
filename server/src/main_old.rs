mod input;

use std::io::{self, Read};
use std::net::{TcpListener, TcpStream};
use std::thread;
use byteorder::{LittleEndian, ReadBytesExt}; // for reading data in Little Endian
// Enum for Sensors
#[repr(u8)]
#[derive(Debug)]
enum Sensors {
    Lidar = 0,
    Imu = 1,
    EncoderMotor = 2,
    Battery = 3,
}

// Enum for ActionTypes
#[repr(u8)]
#[derive(Debug)]
enum ActionType {
    Motor = 0,
    Lidar = 1,
    StopAll = 2,
}

// Enum for PacketType which is a union of Sensors and ActionType
#[derive(Debug)]
enum PacketType {
    Sensor(Sensors),
    Action(ActionType),
}

// Function to handle individual packets from the stream
fn handle_packet(mut stream: &TcpStream) -> io::Result<()> {
    // Buffer to read the packet's header (4 bytes for millis, 1 byte for type, 2 bytes for size)
    let mut header = [0u8; 7];
    if let Err(e) = stream.read_exact(&mut header) {
        if e.kind() == io::ErrorKind::UnexpectedEof {
            return Ok(()); // Gracefully handle stream closure
        }
        return Err(e); // Handle other errors
    }

    // Read millis (u32, 4 bytes)
    let millis = (&header[0..4]).read_u32::<LittleEndian>()?;
    println!("Millis: {}", millis);

    // Read sensor type (u8, 1 byte)
    let packet_type_value = header[4];
    let packet_type = match packet_type_value {
        0 => PacketType::Sensor(Sensors::Lidar),
        1 => PacketType::Sensor(Sensors::Imu),
        2 => PacketType::Sensor(Sensors::EncoderMotor),
        3 => PacketType::Sensor(Sensors::Battery),
        _ => {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "Invalid sensor type"));
        }
    };

    // Read the size of the data (u16, 2 bytes)
    let size = (&header[5..7]).read_u16::<LittleEndian>()?;

    // Read the actual data based on size
    let mut data = vec![0u8; size as usize];
    stream.read_exact(&mut data)?;

    // Extract fields directly from the byte vector
    if size >= 5 {
        let sync_quality = data[0];
        let angle_q6_checkbit = u16::from_le_bytes([data[1], data[2]]);
        let distance_q2 = u16::from_le_bytes([data[3], data[4]]);

        // Perform the operations
        let scan_completed = (sync_quality & 0b10000000) != 0; // Extract syncbit
        let distance_mm = (distance_q2 as f32) * 0.25;
        let angle_deg = ((angle_q6_checkbit >> 1) as f32) * 0.015625; // Shift and scale
        let quality = (sync_quality >> 2) & 0x3F; // Extract the 6-bit quality

        // Print the results
        println!("Packet Type: {:?}", packet_type);
        println!("Scan Completed: {}", scan_completed);
        println!("Distance (mm): {:.2}", distance_mm);
        println!("Angle (deg): {:.2}", angle_deg);
        println!("Quality: {}", quality);
    } else {
        println!("Received packet with insufficient data size: {}", size);
    }

    Ok(())
}

// Function to handle a connection
fn handle_connection(stream: TcpStream) {
    println!("New connection from {}", stream.peer_addr().unwrap());

    loop {
        if let Err(e) = handle_packet(&stream) {
            eprintln!("Error processing packet: {}", e);
            break;
        }
    }

    println!("Connection closed: {}", stream.peer_addr().unwrap());
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
    input::window();
    start_server("0.0.0.0:12345")
}
