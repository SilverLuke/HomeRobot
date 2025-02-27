#include <gtest/gtest.h>
#include "communication/protocol.h"

class ProtocolTest : public ::testing::Test {
  protected:
    Protocol *proto;
    WiFiClient client;

  void SetUp() {
    proto = new Protocol();
    // Initialize any necessary members here
    client = WiFiClient();
    // Mock or set up any dependencies if needed
  }

  void TearDown() {
    delete proto;
    // Cleanup any member variables
  }

  // Test cases follow
};

TEST_F(ProtocolTest, Constructor) {
  EXPECT_FALSE(proto->wifi_client.connected());
  EXPECT_EQ(proto->wifi_client.status(), WL_IDLE);

  // Verify size information is printed correctly
  Serial.print("char: ");
  Serial.println(sizeof(char));
  Serial.print("int: ");
  Serial.println(sizeof(int));
  Serial.print("long: ");
  Serial.println(sizeof(long));
  Serial.print("float: ");
  Serial.println(sizeof(float));
  Serial.print("double: ");
  Serial.println(sizeof(double));
  Serial.print("address: ");
  Serial.println(sizeof(void*));

  // Additional test logic here
}

TEST_F(ProtocolTest, Restart) {
  // Test restart functionality
  EXPECT_FALSE(proto->wifi_client.connected());

  proto->restart();

  EXPECT_TRUE(proto->wifi_client.connected());
  EXPECT_EQ(proto->wifi_client.status(), WL_CONNECTED);

  // Verify logging during restart
  EXPECT_LOG_MESSAGE("Connected to server");
  EXPECT_LOG_MESSAGE("Disconnected from server");
}

TEST_F(ProtocolTest, PrefixSize) {
  size_t expected_prefix = sizeof(uint32_t) + sizeof(uint8_t) + sizeof(uint16_t);
  EXPECT_EQ(proto->prefix_size(), expected_prefix);

  // Add more test logic for prefix calculation
}

TEST_F(ProtocolTest, SendPacketSuccess) {
  // Setup: Ensure client is connected and buffer is ready
  EXPECT_TRUE(proto->wifi_client.connected());

  // Create a sample packet
  HomeRobotPacket packet;
  packet.sequence_millis = 123456;
  packet.type.send = SendPacketType::TX_LIDAR;
  packet.size = 100;

  size_t total_size = proto->prefix_size() + packet.size;
  EXPECT_NOTHING(proto->SendPacket(packet));

  // Verify data sent
  EXPECT_EQ(total_size, proto->wifi_client.write(...));

  // Additional verification steps for successful send
}

TEST_F(ProtocolTest, SendPacketFailure) {
  // Setup: Client not connected
  proto->restart();
  EXPECT_FALSE(proto->wifi_client.connected());

  // Attempt to send packet
  HomeRobotPacket packet;
  size_t total_size = proto->prefix_size() + 100; // Assume data

  bool result = proto->SendPacket(packet);
  EXPECT_FALSE(result);

  // Verify error messages and client status
}

TEST_F(ProtocolTest, SendLidarPacket) {
  // Setup: Ensure buffer is ready and packet can be sent
  EXPECT_TRUE(proto->wifi_client.connected());

  // Prepare a complete lidar packet
  HomeRobotPacket packet;
  packet.sequence_millis = 123456;
  packet.type.send = SendPacketType::TX_LIDAR;
  packet.size = 100;

  size_t total_size = proto->PrefixSize() + packet.size;
  bool success = proto->sendLidarPacket();

  // Verify successful send
  EXPECT_TRUE(success);

  // Additional checks for lidar buffer and data
}

TEST_F(ProtocolTest, AddLidarPacket) {
  // Setup: Ensure buffer is empty
  proto->lidar_buffer_end = 0;

  // Prepare small chunk of data to add
  uint8_t* test_data = new uint8_t[10] {0x01, 0x02, 0x03};
  size_t length = sizeof(test_data);

  proto->AddLidarPacket(test_data, length);

  // Verify successful addition and buffer state
  EXPECT_EQ(proto->lidar_buffer_end, length);

  delete[] test_data;
}

TEST_F(ProtocolTest, AddLidarPacketWithOverflow) {
  // Setup: Buffer nearly full
  proto->lidar_buffer_start = 240;
  proto->lidar_buffer_end = MAX_LIDAR_BUFFER_SIZE - 5;

  // Prepare data that would overflow
  uint8_t* test_data = new uint8_t[20] {0x44, 0x55};
  size_t length = sizeof(test_data);

  bool result = proto->addLidarPacket(test_data, length);

  // Verify buffer full condition
  EXPECT_FALSE(result);

  // Ensure lidar_buffer_start and end are updated correctly
  EXPECT_EQ(proto->lidar_buffer_end, MAX_LIDAR_BUFFER_SIZE);
  EXPECT_EQ(proto->buffer_start_millis, 0);

  delete[] test_data;
}

TEST_F(ProtocolTest, AddLidarPacketCircularBuffer) {
  // Setup: Wrap around the buffer
  proto->lidar_buffer_start = MAX_LIDAR_BUFFER_SIZE - length;

  // Prepare data and add to buffer
  uint8_t* test_data = new uint8_t[20] {0x44, 0x55};
  size_t length = sizeof(test_data);

  bool result = proto->addLidarPacket(test_data, length);

  // Verify circular addition
  EXPECT_TRUE(result);

  // Check both parts of the buffer
  // First part at the end of buffer
  EXPECT_EQ(memcmp(proto->lidar_buffer + (MAX_LIDAR_BUFFER_SIZE - length), test_data, length) , 0);
  // Second part at the beginning of buffer
  EXPECT_EQ(memcmp(proto->lidar_buffer, test_data, length) , 0);

  delete[] test_data;
}

TEST_F(ProtocolTest, BufferResetAfterSend) {
  // Setup: Ensure buffer is filled and send it
  proto->lidar_buffer_start = 10;
  proto->lidar_buffer_end = MAX_LIDAR_BUFFER_SIZE - 5;

  // Prepare data that would fill the buffer
  uint8_t* test_data = new uint8_t[15] {0x11, 0x22, 0x33};
  size_t length = sizeof(test_data);

  bool result = proto->addLidarPacket(test_data, length);

  // Verify send and buffer reset
  EXPECT_FALSE(result);
  EXPECT_EQ(proto->lidar_buffer_end, MAX_LIDAR_BUFFER_SIZE);
  EXPECT_EQ(proto->buffer_start_millis, 0);

  delete[] test_data;
}

// Add more test cases as needed

int main(int argc, char **argv) {
  Testing::Init(argv, argv+1, nullptr, nullptr, false);
  return RUN_ALL_TESTS();
}