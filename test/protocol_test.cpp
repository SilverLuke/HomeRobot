#include <Arduino.h>
#include <gtest/gtest.h>
#include "communication/protocol.h"


TEST(addLidarPacket, TestBufferEmpty) {
  // Set buffer start and end to 0 (empty)
  Protocol::lidar_buffer_start = 0;
  Protocol::lidar_buffer_end = 0;

  // Prepare sample data
  uint8_t* packet = new uint8_t[10] {0x01, 0x02, 0x03};
  size_t length = sizeof(packet);

  bool result = Protocol::addLidarPacket(packet, length);

  // Verify buffer content
  EXPECT_EQ(memcmp(Protocol::lidar_buffer, packet, length), 0);
  EXPECT_EQ(Protocol::lidar_buffer_start, 0);
  EXPECT_EQ(Protocol::lidar_buffer_end, length);

  delete[] packet;
}

TEST(addLidarPacket, TestBufferFull) {
  // Set buffer start and end to 0 (empty)
  Protocol::lidar_buffer_start = 0;
  Protocol::lidar_buffer_end = 0;

  // Prepare sample data
  uint8_t* packet = new uint8_t[1500] {0x01, 0x02, 0x03};
  size_t length = sizeof(packet);

  bool result = Protocol::addLidarPacket(packet, length);

  // Verify buffer content
  EXPECT_EQ(memcmp(Protocol::lidar_buffer, packet, length), 0);
  EXPECT_EQ(Protocol::lidar_buffer_start, 0);
  EXPECT_EQ(Protocol::lidar_buffer_end, length);

  delete[] packet;
}

TEST(addLidarPacket, TestWrapping) {
  // Assume buffer is nearly full
  Protocol::lidar_buffer_start = 240;
  Protocol::lidar_buffer_end = 255;

  // Prepare data to wrap around
  uint8_t* packet = new uint8_t[20] {0x44, 0x55};
  size_t length = sizeof(packet);

  bool result = Protocol::addLidarPacket(packet, length);

  // Verify first part in buffer
  EXPECT_EQ(memcmp(Protocol::lidar_buffer + 240, packet, 20) , 0);
  // Verify the wrap-around part at the beginning of the buffer
  EXPECT_EQ(memcmp(Protocol::lidar_buffer, packet, 20) , 0);

  delete[] packet;
}

TEST(addLidarPacket, TestBufferFull) {
  // Initialize buffer as full
  Protocol::lidar_buffer_start = 10;
  Protocol::lidar_buffer_end = MAX_LIDAR_BUFFER_SIZE - 5;

  // Prepare data that would exceed buffer size
  uint8_t* packet = new uint8_t[15] {0x11, 0x22, 0x33};
  size_t length = sizeof(packet);

  bool result = Protocol::addLidarPacket(packet, length);

  EXPECT_FALSE(result);
  EXPECT_EQ(Protocol::lidar_buffer_start, 10);
  EXPECT_EQ(Protocol::lidar_buffer_end, MAX_LIDAR_BUFFER_SIZE);

  delete[] packet;
}


#if defined(ARDUINO)
#include <Arduino.h>

void setup()
{
    // should be the same value as for the `test_speed` option in "platformio.ini"
    // default value is test_speed=115200
    Serial.begin(115200);

    ::testing::InitGoogleTest();
}

void loop()
{
	// Run tests
	if (RUN_ALL_TESTS())
	;

	// sleep 1 sec
	delay(1000);
}

#else
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
	if (RUN_ALL_TESTS())
	;
	// Always return zero-code and allow PlatformIO to parse results
	return 0;
}
#endif
