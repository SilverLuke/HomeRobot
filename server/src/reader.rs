use crate::constants::BUFFER_SIZE;
use crate::homerobot::RobotToServerMessage;
use crate::stats::Stats;
use circular_buffer::CircularBuffer;
use prost::Message;
use std::io;
use std::io::{Read, Write};
use std::sync::atomic::Ordering;
use std::sync::Arc;

pub struct ProtocolManager<S: Read + Write> {
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
        // 1. Ensure we have the 2-byte length prefix
        if self.read_buffer.len() < 2 {
            self.do_read()?;
            if self.read_buffer.len() < 2 {
                return Ok(None);
            }
        }

        // 2. Peek at the length prefix (Big-Endian u16)
        let msg_len = {
            let b0 = *self.read_buffer.get(0).expect("buffer length verified");
            let b1 = *self.read_buffer.get(1).expect("buffer length verified");
            u16::from_be_bytes([b0, b1]) as usize
        };

        // 3. Sanity check: message must fit in our buffer
        if msg_len > BUFFER_SIZE - 2 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Message size {} exceeds buffer capacity", msg_len),
            ));
        }

        // 4. Ensure we have the full message body
        if self.read_buffer.len() < 2 + msg_len {
            self.do_read()?;
            if self.read_buffer.len() < 2 + msg_len {
                return Ok(None);
            }
        }

        // 5. Consume header and read body
        let mut _header = [0u8; 2];
        self.read_buffer.read_exact(&mut _header)?;

        let mut msg_bytes = vec![0u8; msg_len];
        self.read_buffer.read_exact(&mut msg_bytes)?;

        // 6. Decode Protobuf message
        RobotToServerMessage::decode(&msg_bytes[..])
            .map(Some)
            .map_err(|e| {
                io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("Protobuf decode error: {}", e),
                )
            })
    }

    pub fn send_packet(&mut self, packet: &[u8]) -> Result<(), std::io::Error> {
        let bytes_written = self.stream.write(packet)?;
        self.stats
            .total_tx
            .fetch_add(bytes_written, Ordering::SeqCst);
        self.stream.flush()?;
        Ok(())
    }

    fn do_read(&mut self) -> io::Result<usize> {
        let free_space = BUFFER_SIZE - self.read_buffer.len();
        if free_space == 0 {
            return Err(io::Error::new(io::ErrorKind::Other, "Buffer overflow"));
        }

        let mut buffer = [0u8; 1024];
        let read_limit = std::cmp::min(buffer.len(), free_space);

        match self.stream.read(&mut buffer[..read_limit]) {
            Ok(0) => Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "Connection closed",
            )),
            Ok(read_bytes) => {
                self.stats.total_rx.fetch_add(read_bytes, Ordering::SeqCst);
                self.read_buffer.extend(&buffer[..read_bytes]);
                Ok(read_bytes)
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => Ok(0),
            Err(error) => Err(error),
        }
    }
}
