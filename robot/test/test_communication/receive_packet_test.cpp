#include "fake_client.cpp"

TEST(ProtocolReceivePacket, PartialThenComplete) {
  FakeClient* client = new FakeClient();
  Protocol protocol(client);

  // Prepare packet with 3 bytes payload
  const size_t PAYLOAD_SIZE = 3;
  uint8_t payload[PAYLOAD_SIZE] = {0xDE, 0xAD, 0xBE};
  auto pkt = make_packet(1000, ReceivePacketType::RX_REQUEST, PAYLOAD_SIZE, payload);

  // Simulate reads: 3 bytes, then 4 bytes (header complete), then 3 bytes (payload)
  const size_t CHUNKS_SIZE = 3;
  uint8_t chunks[CHUNKS_SIZE] = {3, 4, 3};
  client->setInput(pkt, Protocol::HeaderSize() + PAYLOAD_SIZE, chunks, CHUNKS_SIZE);

  // first call: less than header
  EXPECT_FALSE(protocol.ReceivePacket());
  EXPECT_EQ(protocol.stats->rx_bytes, chunks[0]);
  EXPECT_EQ(protocol.stats->tx_bytes, 0);
  EXPECT_EQ(protocol.getUsedBuffer(), chunks[0]);
  // second: header complete, but no payload yet
  EXPECT_FALSE(protocol.ReceivePacket());
  EXPECT_EQ(protocol.stats->rx_bytes, chunks[1] + chunks[0]);
  EXPECT_EQ(protocol.getUsedBuffer(), chunks[1] + chunks[0]);

  EXPECT_TRUE(protocol.hasReadHeader());
  auto [sequence_millis, type, size] = protocol.getParsedHeder();
  EXPECT_EQ(sequence_millis, 1000);
  EXPECT_EQ(type.receive, ReceivePacketType::RX_REQUEST);
  EXPECT_EQ(size, PAYLOAD_SIZE);

  // third: full packet now
  EXPECT_TRUE(protocol.ReceivePacket());
  EXPECT_EQ(protocol.stats->rx_bytes, chunks[2] + chunks[1] + chunks[0]);
  EXPECT_EQ(protocol.getUsedBuffer(), 0);

  EXPECT_EQ(protocol.receive_packet.header.sequence_millis, 1000u);
  EXPECT_EQ(protocol.receive_packet.header.type.receive, ReceivePacketType::RX_REQUEST);
  EXPECT_EQ(protocol.receive_packet.header.size, sizeof(payload));
  ASSERT_NE(protocol.receive_packet.data, nullptr);

  free(pkt);
}

TEST(ProtocolReceivePacket, OversizedPacketRejected) {
  FakeClient* client = new FakeClient();
  Protocol proto(client);

  uint8_t payload[RX_MAX_PACKET_SIZE];
  memset(payload, 0x11, RX_MAX_PACKET_SIZE);
  auto pkt = make_packet(10, ReceivePacketType::RX_REQUEST, RX_MAX_PACKET_SIZE, payload);
  uint8_t per_read[1] = {7};

  client->setInput(pkt, HEADER_SIZE + RX_MAX_PACKET_SIZE, per_read, 1); // read at least header

  // First call reads header
  EXPECT_FALSE(proto.ReceivePacket());
  // Next calls try to read and should detect oversized and reset
  EXPECT_FALSE(proto.ReceivePacket());
}

TEST(ProtocolReceivePacket, ZeroReadReturnsFalse) {
  const auto client = new FakeClient();
  Protocol proto(client);
  uint8_t payload[1] = {};
  client->setInput(payload, 1,  payload, 1);
  EXPECT_FALSE(proto.ReceivePacket());
}

/**
 * This test check if 2 packets are well parsed, with good reads
 */
TEST(ProtocolReceivePacket, TwoPacketsBufferRollover) {
  const auto client = new FakeClient();
  Protocol proto(client);

  uint8_t payload[4] = {1,2,3,4};
  uint8_t payload1[2] = {5,6};
  auto pkt1 = make_packet(1, ReceivePacketType::RX_REQUEST, sizeof(payload), payload);
  auto pkt2 = make_packet(2, ReceivePacketType::RX_REQUEST, sizeof(payload1), payload1);
  // Stream is pkt1 followed by pkt2 in one go (to test memmove rollover)
  uint8_t stream[7 + sizeof(payload) + 7 + sizeof(payload1)];
  memmove(stream, pkt1, 7 + sizeof(payload));
  memmove(stream + 7 + sizeof(payload), pkt2, 7 + sizeof(payload1));

  // The sizeof and the payload are swapped
  uint8_t per_read[4] = {HEADER_SIZE + sizeof(payload), HEADER_SIZE + sizeof(payload1)};
  client->setInput(
    stream,
    HEADER_SIZE*2 + sizeof(payload) + sizeof(payload1),
    per_read,
    4
  );

  // First: should read both, but only one packet is delivered per call
  EXPECT_TRUE(proto.ReceivePacket());
  EXPECT_EQ(proto.receive_packet.header.sequence_millis, 1u);
  EXPECT_EQ(proto.receive_packet.header.size, sizeof(payload));
  EXPECT_EQ(0, memcmp(proto.receive_packet.data, payload, sizeof(payload)));

  // Second call should deliver the second packet from leftover bytes
  EXPECT_TRUE(proto.ReceivePacket());
  EXPECT_EQ(proto.receive_packet.header.sequence_millis, 2u);
  EXPECT_EQ(proto.receive_packet.header.size, sizeof(payload1));
  EXPECT_EQ(0, memcmp(proto.receive_packet.data, payload1, sizeof(payload1)));
}
/**
 *  This test check if the read split each packet in half the program is capable to read them
 */
TEST(ProtocolReceivePacket, FourReadsRequired) {
  const auto client = new FakeClient();
  Protocol proto(client);

  uint8_t payload[4] = {1,2,3,4};
  uint8_t payload1[2] = {5,6};
  auto pkt1 = make_packet(1, ReceivePacketType::RX_REQUEST, sizeof(payload), payload);
  auto pkt2 = make_packet(2, ReceivePacketType::RX_REQUEST, sizeof(payload1), payload1);
  // Stream is pkt1 followed by pkt2 in one go (to test memmove rollover)
  const size_t stream_size = HEADER_SIZE + sizeof(payload) + HEADER_SIZE + sizeof(payload1);
  uint8_t stream[stream_size];
  memmove(stream, pkt1, HEADER_SIZE + sizeof(payload));
  memmove(stream + HEADER_SIZE + sizeof(payload), pkt2, HEADER_SIZE + sizeof(payload1));

  // The sizeof and the payload are swapped
  uint8_t per_read[4] = {sizeof(payload), HEADER_SIZE, sizeof(payload1), HEADER_SIZE};

  client->setInput(
    stream,
    stream_size,
    per_read,
    4
  );

  // First read() read only a part of the first packet
  EXPECT_FALSE(proto.ReceivePacket());
  // This read() end the first packet
  EXPECT_TRUE(proto.ReceivePacket());
  EXPECT_EQ(proto.receive_packet.header.sequence_millis, 1u);
  EXPECT_EQ(proto.receive_packet.header.size, sizeof(payload));
  EXPECT_EQ(0, memcmp(proto.receive_packet.data, payload, sizeof(payload)));

  // As for the first packet the first read (of the second packet) read a part of it
  EXPECT_FALSE(proto.ReceivePacket());
  EXPECT_TRUE(proto.ReceivePacket());
  EXPECT_EQ(proto.receive_packet.header.sequence_millis, 2u);
  EXPECT_EQ(proto.receive_packet.header.size, sizeof(payload1));
  EXPECT_EQ(0, memcmp(proto.receive_packet.data, payload1, sizeof(payload1)));
}



#include <Arduino.h>

void setup()
{
  // should be the same value as for the `test_speed` option in "platformio.ini"
  // default value is test_speed=115200
  Serial.begin(115200);

  ::testing::InitGoogleTest();
  // if you plan to use GMock, replace the line above with
  // ::testing::InitGoogleMock();
}

void loop()
{
  // Run tests
  if (RUN_ALL_TESTS())
    ;

  // sleep for 1 sec
  delay(1000);
}
// #else
// int main(int argc, char **argv)
// {
//   ::testing::InitGoogleTest(&argc, argv);
//   // if you plan to use GMock, replace the line above with
//   // ::testing::InitGoogleMock(&argc, argv);
//
//   if (RUN_ALL_TESTS())
//     ;
//
//   // Always return zero-code and allow PlatformIO to parse results
//   return 0;
// }
// #endif