#include <Arduino.h>
#include <arpa/inet.h>
#include <errno.h>
#include <gtest/gtest.h>
#include <sys/unistd.h>

#include <cstdint>
#include <cstring>

#include "../../src/communication/protocol.h"
#include "../../src/communication/net_client.h"
#include "../../src/communication/packet_types.h"

const uint8_t HEADER_SIZE = 7;

class FakeClient : public NetClient {
public:
  uint8_t* input;
  size_t input_size;

  uint8_t* chunks;
  size_t chunks_size;

  size_t in_pos = 0;
  size_t chunk_pos = 0;
  bool connected_flag = true;
  uint8_t* output;

  void setInput(
    uint8_t* data,
    size_t data_length,
    uint8_t* per_read,
    size_t per_read_size
  ) {
    input = data;
    input_size = data_length;
    chunks = per_read;
    chunks_size = per_read_size;
    in_pos = 0;
    chunk_pos = 0;
  }

  // NetClient impl
  std::size_t read(uint8_t* buffer, std::size_t size) override {
    if (in_pos >= input_size) return 0;
    size_t remaining = input_size - in_pos;
    size_t to_read = std::min<size_t>(size, remaining);
    if (chunk_pos < chunks_size) {
      to_read = std::min<size_t>(to_read, chunks[chunk_pos]);
      chunk_pos++;
    }
    if (to_read == 0) return 0;
    memcpy(buffer, input + in_pos, to_read);
    in_pos += to_read;
    return to_read;
  }

  std::size_t write(const uint8_t* buffer, std::size_t size) override {
    return 0;
    // output.insert(output.end(), buffer, buffer + size);
    // return size;
  }

  bool connected() override { return connected_flag; }
  bool connect(const char*, uint16_t) override { connected_flag = true; return true; }
  void stop() override { connected_flag = false; }
  void flush() override {}
};

static uint8_t* make_packet(uint32_t millis, ReceivePacketType type, uint16_t payload_size, const uint8_t* payload) {
  uint8_t* v = (uint8_t*) malloc(100);
  uint32_t m_be = htonl(millis);
  memcpy(v, &m_be, 4);
  v[4] = static_cast<uint8_t>(type);
  uint16_t s_be = htons(payload_size);
  memcpy(v + 5, &s_be, 2);
  if (payload_size > 0) {
    memcpy(v + HEADER_SIZE, payload, payload_size);
  }
  return v;
}

TEST(ProtocolReceivePacket, PartialThenComplete) {
  FakeClient* client = new FakeClient();
  Protocol protocol(client);

  // Prepare packet with 3 bytes payload
  uint8_t payload[3] = {0xDE, 0xAD, 0xBE};
  auto pkt = make_packet(100, ReceivePacketType::RX_REQUEST, sizeof(payload), payload);

  // Simulate reads: 3 bytes, then 4 bytes (header complete), then 3 bytes (payload)
  uint8_t chunks[3] = {3, 4, 3};
  client->setInput(pkt, HEADER_SIZE+3, chunks, 3);

  // first call: less than header
  EXPECT_FALSE(protocol.ReceivePacket());
  // second: header complete, but no payload yet
  EXPECT_FALSE(protocol.ReceivePacket());
  // third: full packet now
  EXPECT_TRUE(protocol.ReceivePacket());

  EXPECT_EQ(protocol.receive_packet.header.sequence_millis, 100u);
  EXPECT_EQ(protocol.receive_packet.header.type.receive, ReceivePacketType::RX_REQUEST);
  EXPECT_EQ(protocol.receive_packet.header.size, sizeof(payload));
  ASSERT_NE(protocol.receive_packet.data, nullptr);
  EXPECT_EQ(0, memcmp(protocol.receive_packet.data, payload, sizeof(payload)));

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
  uint8_t payload[1] = {0x11};
  client->setInput(payload, 0,  payload, 0);
  EXPECT_FALSE(proto.ReceivePacket());
}
// /**
//  * This test check if 2 packets are well parsed, with good reads
//  */
// TEST(ProtocolReceivePacket, TwoPacketsBufferRollover) {
//   const auto client = new FakeClient();
//   Protocol proto(client);
//
//   uint8_t payload[4] = {1,2,3,4};
//   uint8_t payload1[2] = {5,6};
//   auto pkt1 = make_packet(1, ReceivePacketType::RX_REQUEST, sizeof(payload), payload);
//   auto pkt2 = make_packet(2, ReceivePacketType::RX_REQUEST, sizeof(payload1), payload1);
//   // Stream is pkt1 followed by pkt2 in one go (to test memmove rollover)
//   uint8_t stream[7 + sizeof(payload) + 7 + sizeof(payload1)];
//   memmove(stream, pkt1, 7 + sizeof(payload));
//   memmove(stream + 7 + sizeof(payload), pkt2, 7 + sizeof(payload1));
//
//   // The sizeof and the payload are swapped
//   uint8_t per_read[4] = {HEADER_SIZE + sizeof(payload), HEADER_SIZE + sizeof(payload1)};
//   client->setInput(
//     stream,
//     HEADER_SIZE*2 + sizeof(payload) + sizeof(payload1),
//     per_read,
//     4
//   );
//
//   // First: should read both, but only one packet is delivered per call
//   EXPECT_TRUE(proto.ReceivePacket());
//   EXPECT_EQ(proto.receive_packet.header.sequence_millis, 1u);
//   EXPECT_EQ(proto.receive_packet.header.size, sizeof(payload));
//   EXPECT_EQ(0, memcmp(proto.receive_packet.data, payload, sizeof(payload)));
//
//   // Second call should deliver the second packet from leftover bytes
//   EXPECT_TRUE(proto.ReceivePacket());
//   EXPECT_EQ(proto.receive_packet.header.sequence_millis, 2u);
//   EXPECT_EQ(proto.receive_packet.header.size, sizeof(payload1));
//   EXPECT_EQ(0, memcmp(proto.receive_packet.data, payload1, sizeof(payload1)));
// }
// /**
//  *  This test check if the read split each packet in half the program is capable to read them
//  */
// TEST(ProtocolReceivePacket, FourReadsRequired) {
//   const auto client = new FakeClient();
//   Protocol proto(client);
//
//   uint8_t payload[4] = {1,2,3,4};
//   uint8_t payload1[2] = {5,6};
//   auto pkt1 = make_packet(1, ReceivePacketType::RX_REQUEST, sizeof(payload), payload);
//   auto pkt2 = make_packet(2, ReceivePacketType::RX_REQUEST, sizeof(payload1), payload1);
//   // Stream is pkt1 followed by pkt2 in one go (to test memmove rollover)
//   const size_t stream_size = HEADER_SIZE + sizeof(payload) + HEADER_SIZE + sizeof(payload1);
//   uint8_t stream[stream_size];
//   memmove(stream, pkt1, HEADER_SIZE + sizeof(payload));
//   memmove(stream + HEADER_SIZE + sizeof(payload), pkt2, HEADER_SIZE + sizeof(payload1));
//
//   // The sizeof and the payload are swapped
//   uint8_t per_read[4] = {sizeof(payload), HEADER_SIZE, sizeof(payload1), HEADER_SIZE};
//
//   client->setInput(
//     stream,
//     stream_size,
//     per_read,
//     4
//   );
//
//   // First read() read only a part of the first packet
//   EXPECT_FALSE(proto.ReceivePacket());
//   // This read() end the first packet
//   EXPECT_TRUE(proto.ReceivePacket());
//   EXPECT_EQ(proto.receive_packet.header.sequence_millis, 1u);
//   EXPECT_EQ(proto.receive_packet.header.size, sizeof(payload));
//   EXPECT_EQ(0, memcmp(proto.receive_packet.data, payload, sizeof(payload)));
//
//   // As for the first packet the first read (of the second packet) read a part of it
//   EXPECT_FALSE(proto.ReceivePacket());
//   EXPECT_TRUE(proto.ReceivePacket());
//   EXPECT_EQ(proto.receive_packet.header.sequence_millis, 2u);
//   EXPECT_EQ(proto.receive_packet.header.size, sizeof(payload1));
//   EXPECT_EQ(0, memcmp(proto.receive_packet.data, payload1, sizeof(payload1)));
// }



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