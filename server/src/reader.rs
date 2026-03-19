use crate::homerobot::RobotToServerMessage;
use crate::constants::BUFFER_SIZE;
use crate::stats::Stats;
use circular_buffer::CircularBuffer;
use std::io;
use std::io::{Read, Write};
use std::sync::atomic::Ordering;
use std::sync::Arc;
use prost::Message;

pub struct ProtocolManager<S: Read + Write>{
    stream: S,
    read_buffer: Box<CircularBuffer<BUFFER_SIZE, u8>>,
    stats: Arc<Stats>,
}

impl<S: Read + Write> ProtocolManager<S> {
    pub(crate) fn new(stream: S, stats: Arc<Stats>) -> ProtocolManager<S> {
        ProtocolManager {
            stream,
            read_buffer: CircularBuffer::<BUFFER_SIZE, u8>::boxed(),
            stats,
        }
    }

    pub(crate) fn read_message(&mut self) -> io::Result<Option<RobotToServerMessage>> {
        self.do_read()?;

        if self.read_buffer.len() < 2 {
            return Ok(None);
        }

        // Peek at the length prefix (2 bytes)
        let mut len_bytes = [0u8; 2];
        for i in 0..2 {
            len_bytes[i] = *self.read_buffer.get(i).unwrap();
        }
        let msg_len = u16::from_be_bytes(len_bytes) as usize;

        if self.read_buffer.len() < 2 + msg_len {
            return Ok(None);
        }

        // Consume the length prefix
        self.read_buffer.read_exact(&mut len_bytes).unwrap();

        // Read the message body
        let mut msg_bytes = vec![0u8; msg_len];
        self.read_buffer.read_exact(&mut msg_bytes).unwrap();

        match RobotToServerMessage::decode(&msg_bytes[..]) {
            Ok(msg) => {
                // println!(
                //     "Received Protobuf message! Seq: {}",
                //     msg.sequence_millis
                // );
                Ok(Some(msg))
            }
            Err(e) => {
                Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("Failed to decode Protobuf: {}", e),
                ))
            }
        }
    }

    pub fn send_packet(&mut self, packet: &[u8]) -> Result<(), std::io::Error> {
        let bytes_written = self.stream.write(packet)?;
        self.stream.flush()?;

        self.stats.total_tx.fetch_add(bytes_written, Ordering::SeqCst);

        Ok(())
    }

    fn do_read(&mut self) -> io::Result<usize> {
        let mut buffer = [0u8; 1024];
        match self.stream.read(&mut buffer) {
            Ok(0) => Err(io::Error::new(io::ErrorKind::UnexpectedEof, "Connection closed")),
            Ok(read_bytes) => {
                self.stats.total_rx.fetch_add(read_bytes, Ordering::SeqCst);
                let free_space = BUFFER_SIZE - self.read_buffer.len();
                if free_space >= read_bytes {
                    self.read_buffer.extend(&buffer[..read_bytes]);
                    Ok(read_bytes)
                } else {
                    Err(io::Error::new(io::ErrorKind::Other, "Buffer overflow"))
                }
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                Ok(0)
            }
            Err(error) => Err(error),
        }
    }
}
