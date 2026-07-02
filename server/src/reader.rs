use crate::constants::BUFFER_SIZE;
use crate::homerobot::RobotToServerMessage;
use circular_buffer::CircularBuffer;
use prost::Message;
use std::io;
use std::io::{Read, Write};

pub struct ProtocolManager<S: Read + Write> {
    stream: S,
    read_buffer: Box<CircularBuffer<BUFFER_SIZE, u8>>,
}

impl<S: Read + Write> ProtocolManager<S> {
    pub(crate) fn new(stream: S) -> ProtocolManager<S> {
        ProtocolManager {
            stream,
            read_buffer: CircularBuffer::<BUFFER_SIZE, u8>::boxed(),
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
                self.read_buffer.extend(&buffer[..read_bytes]);
                Ok(read_bytes)
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => Ok(0),
            Err(error) => Err(error),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::homerobot::robot_to_server_message::Payload;
    use crate::homerobot::{BatteryStatus, RobotToServerMessage};
    use std::collections::VecDeque;

    /// Test stream: serves one queued chunk per read call, then WouldBlock;
    /// an empty chunk simulates EOF (read returning 0).
    struct ChunkedStream {
        chunks: VecDeque<Vec<u8>>,
    }

    impl ChunkedStream {
        fn new<I: IntoIterator<Item = Vec<u8>>>(chunks: I) -> Self {
            Self { chunks: chunks.into_iter().collect() }
        }
    }

    impl Read for ChunkedStream {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            match self.chunks.pop_front() {
                Some(chunk) => {
                    buf[..chunk.len()].copy_from_slice(&chunk);
                    Ok(chunk.len())
                }
                None => Err(io::Error::new(io::ErrorKind::WouldBlock, "no data")),
            }
        }
    }

    impl Write for ChunkedStream {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            Ok(buf.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    fn frame(msg: &RobotToServerMessage) -> Vec<u8> {
        let mut body = Vec::new();
        msg.encode(&mut body).unwrap();
        let mut packet = (body.len() as u16).to_be_bytes().to_vec();
        packet.extend(body);
        packet
    }

    fn battery_msg(percentage: u32) -> RobotToServerMessage {
        RobotToServerMessage {
            sequence_millis: 1,
            payload: Some(Payload::Battery(BatteryStatus {
                percentage,
                voltage_mv: 3700,
                raw_value: 0,
            })),
        }
    }

    fn manager(chunks: Vec<Vec<u8>>) -> ProtocolManager<ChunkedStream> {
        ProtocolManager::new(ChunkedStream::new(chunks))
    }

    #[test]
    fn reassembles_a_message_split_across_reads() {
        let packet = frame(&battery_msg(80));
        // Split inside the 2-byte length prefix: read_message performs at most
        // one extra read per missing part, so this forces a second call.
        let (first, second) = packet.split_at(1);
        let mut protocol = manager(vec![first.to_vec(), second.to_vec()]);

        // First call sees only a partial frame.
        assert!(protocol.read_message().unwrap().is_none());
        let msg = protocol.read_message().unwrap().expect("message after second chunk");
        assert!(matches!(msg.payload, Some(Payload::Battery(b)) if b.percentage == 80));
    }

    #[test]
    fn returns_two_messages_arriving_in_one_read() {
        let mut chunk = frame(&battery_msg(10));
        chunk.extend(frame(&battery_msg(20)));
        let mut protocol = manager(vec![chunk]);

        let first = protocol.read_message().unwrap().unwrap();
        let second = protocol.read_message().unwrap().unwrap();
        assert!(matches!(first.payload, Some(Payload::Battery(b)) if b.percentage == 10));
        assert!(matches!(second.payload, Some(Payload::Battery(b)) if b.percentage == 20));
    }

    #[test]
    fn rejects_length_prefix_exceeding_buffer() {
        let oversized = (BUFFER_SIZE as u16).to_be_bytes().to_vec();
        let mut protocol = manager(vec![oversized]);
        let err = protocol.read_message().unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::InvalidData);
    }

    #[test]
    fn rejects_undecodable_payload() {
        // Valid length prefix, garbage protobuf body (truncated varint).
        let mut protocol = manager(vec![vec![0x00, 0x02, 0xFF, 0xFF]]);
        let err = protocol.read_message().unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::InvalidData);
    }

    #[test]
    fn reports_eof_as_error() {
        let mut protocol = manager(vec![vec![]]); // read returning 0 bytes
        let err = protocol.read_message().unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::UnexpectedEof);
    }

    #[test]
    fn would_block_yields_no_message() {
        let mut protocol = manager(vec![]);
        assert!(protocol.read_message().unwrap().is_none());
    }
}
