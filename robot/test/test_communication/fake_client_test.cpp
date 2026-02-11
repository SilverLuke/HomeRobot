#include "fake_client.cpp"

TEST(FakeClient, ReadSafe ) {
  auto client = new FakeClient();
  constexpr size_t payload_size = 3;
  const uint8_t payload[payload_size] = {0xC0, 0xFF, 0xE0};
  const auto pkt = make_packet(255, ReceivePacketType::RX_REQUEST, sizeof(payload), payload);

  // Simulate reads: 3 bytes, then 4 bytes (header complete), then 3 bytes
  // (payload)
  constexpr size_t CHUNKS_SIZE = 3;
  uint8_t chunks[CHUNKS_SIZE] = {3, 4, 3};
  client->setInput(pkt, HEADER_SIZE+payload_size, chunks, CHUNKS_SIZE);

  uint8_t buffer[20] = {};
  size_t read = client->read(buffer, HEADER_SIZE+payload_size);
  EXPECT_EQ(read, chunks[0]);
  EXPECT_EQ(buffer[0], 0x00);
  EXPECT_EQ(buffer[1], 0x00);
  EXPECT_EQ(buffer[2], 0x00);

  read = client->read(buffer, HEADER_SIZE+payload_size);
  EXPECT_EQ(read, chunks[1]);
  EXPECT_EQ(buffer[0], 0xFF);
  EXPECT_EQ(buffer[1], 0x08);
  EXPECT_EQ(buffer[2], 0x00);
  EXPECT_EQ(buffer[3], 0x03);

  read = client->read(buffer, HEADER_SIZE+payload_size);
  EXPECT_EQ(read, chunks[2]);
  EXPECT_EQ(buffer[0], payload[0]);
  EXPECT_EQ(buffer[1], payload[1]);
  EXPECT_EQ(buffer[2], payload[2]);
}


TEST(FakeClient, ReadMoreThanBuffer) {
  auto client = new FakeClient();
  constexpr size_t payload_size = 3;
  const uint8_t payload[payload_size] = {0xDE, 0xAD, 0xBE};
  const auto pkt = make_packet(255, ReceivePacketType::RX_REQUEST, sizeof(payload), payload);

  // Simulate reads: 3 bytes, then 4 bytes (header complete), then 3 bytes
  // (payload)
  constexpr size_t CHUNKS_SIZE = 3;
  uint8_t chunks[CHUNKS_SIZE] = {3, 4, 10};
  client->setInput(pkt, HEADER_SIZE+payload_size, chunks, CHUNKS_SIZE);

  uint8_t buffer[20] = {};
  size_t read = client->read(buffer, HEADER_SIZE+payload_size);
  EXPECT_EQ(read, chunks[0]);
  EXPECT_EQ(buffer[0], 0x00);
  EXPECT_EQ(buffer[1], 0x00);
  EXPECT_EQ(buffer[2], 0x00);

  read = client->read(buffer, HEADER_SIZE+payload_size);
  EXPECT_EQ(read, chunks[1]);
  EXPECT_EQ(buffer[0], 0xFF);
  EXPECT_EQ(buffer[1], 0x08);
  EXPECT_EQ(buffer[2], 0x00);
  EXPECT_EQ(buffer[3], 0x03);

  read = client->read(buffer, HEADER_SIZE+payload_size);
  EXPECT_EQ(read, 3);
  EXPECT_EQ(buffer[0], payload[0]);
  EXPECT_EQ(buffer[1], payload[1]);
  EXPECT_EQ(buffer[2], payload[2]);


  read = client->read(buffer, HEADER_SIZE+payload_size);
  EXPECT_EQ(read, 0);
}

TEST(FakeClient, ReadSetTo0) {
  const auto client = new FakeClient();

  uint8_t payload[1] = {};
  client->setInput(payload, 1,  payload, 1);

  uint8_t buffer[20] = {0xFF, 0xFF};
  size_t read = client->read(buffer, 10);

  EXPECT_EQ(read, 0);
  EXPECT_EQ(buffer[0], 0xFF);
  EXPECT_EQ(buffer[1], 0xFF);

}