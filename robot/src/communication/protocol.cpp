#include "protocol.h"

#include <WiFi.h>

#include <cstdint>

#include "packet_types.h"
#include "secrets.h"

Protocol::Protocol() {
  // Initialize the Wi-Fi client
  if (WiFi.isConnected()) {
    this->wifi_client = WiFiClient();
    this->connect();
  }

  this->receive_packet.data = this->rx_buffer;
#ifdef SHOW_VAR_SIZE
  Serial.print("bool: ");
  Serial.println(sizeof(bool));
  Serial.print("char: ");
  Serial.println(sizeof(char));
  Serial.print("short: ");
  Serial.println(sizeof(short));
  Serial.print("int: ");
  Serial.println(sizeof(int));
  Serial.print("long: ");
  Serial.println(sizeof(long));
  Serial.print("unsigned long: ");
  Serial.println(sizeof(unsigned long));
  Serial.print("long long: ");
  Serial.println(sizeof(long long));
  Serial.print("float: ");
  Serial.println(sizeof(float));
  Serial.print("double: ");
  Serial.println(sizeof(double));
  Serial.print("long double: ");
  Serial.println(sizeof(long double));
  Serial.print("void*: ");
  Serial.println(sizeof(void*));
  Serial.print("unit8_t*: ");
  Serial.println(sizeof(uint8_t*));
#endif
}

void Protocol::connect() {
  int attempts = 0;
  while (!this->wifi_client.connect(wifi_server_host, wifi_server_port) &&
         attempts < 5) {
    Serial.println("Failed to connect to server");
    attempts++;
    delay(1000);
  }
  Serial.println("Connected to server");
}

bool Protocol::isConnected() { return this->wifi_client.connected(); }

void Protocol::restart() {
  if (this->wifi_client.connected()) {
    this->wifi_client.stop();
  }
  connect();
  this->receive_packet = {};
}

size_t Protocol::HeaderSize() {
  return sizeof(HomeRobotPacket::sequence_millis) +
         sizeof(HomeRobotPacket::type) + sizeof(HomeRobotPacket::size);
}

size_t Protocol::SendPacket(const HomeRobotPacket& packet) {
  if (this->wifi_client.connected()) {
    const size_t constant_offset = Protocol::HeaderSize();

    const size_t totalSize = constant_offset + packet.size;

    memcpy(this->tx_buffer, &packet, constant_offset);

    // Copy the data array if it exists
    if (packet.data && packet.size > 0) {
      memcpy(this->tx_buffer + constant_offset, packet.data, packet.size);
    }

    // Send data
    const size_t bytesWritten =
        this->wifi_client.write(this->tx_buffer, totalSize);
    if (bytesWritten != totalSize) {
      log_e("Error writing packet");
    } else {
      // Serial.println("Data sent");
    }
    return bytesWritten;
  } else {
    log_e("Error client not connected");
  }
  return 0;
}

// void Protocol::SendLidarPacket() {
//   /**
//    * @brief Send the lidar buffer to the PC
//    */
//
//   // HomeRobotPacket packet = {};
//   // packet.sequence_millis = this->buffer_start_millis;
//   // packet.type.send = SendPacketType::TX_LIDAR;
//   // packet.size = this->lidar_buffer_end;
//   // packet.data = this->lidar_buffer;
//   size_t offset = 0;
//
//   // Coping the millis - 4 bytes
//   memcpy(this->lidar_buffer, &this->buffer_start_millis,
//   sizeof(this->buffer_start_millis)); offset +=
//   sizeof(this->buffer_start_millis);
//
//   // Coping the packet type - 1 byte
//   constexpr auto packet = SendPacketType::TX_LIDAR;
//   memcpy(this->lidar_buffer + offset, &packet, sizeof(packet));
//   offset += sizeof(SendPacketType);
//
//   // Coping the data size - 2 bytets
//   this->lidar_buffer_end -= Protocol::HeaderSize();
//   memcpy(this->lidar_buffer + offset, &this->lidar_buffer_end,
//   sizeof(this->lidar_buffer_end)); offset += sizeof(this->lidar_buffer_end);
//   assert(offset == Protocol::HeaderSize());
//
//   log_i("Sending lidar packet with size %d", this->lidar_buffer_end);
//   size_t written = this->wifi_client.write(this->lidar_buffer,
//   this->lidar_buffer_end + offset); if (written != this->lidar_buffer_end +
//   offset) {
//     sleep(2);
//     written = this->wifi_client.write(this->lidar_buffer + written,
//     this->lidar_buffer_end + offset - written);
//   }
//   assert(written == this->lidar_buffer_end + offset);
//
//   this->lidar_buffer_end = HeaderSize();
//   this->buffer_start_millis = 0;
//
// }



#include <arpa/inet.h>

bool Protocol::ReceivePacket() {
  uint8_t prefix =
      this->wifi_client.read(this->rx_buffer, Protocol::HeaderSize());
  if (prefix > 0 && prefix != Protocol::HeaderSize()) {
    log_e("Unable to read full prefix, read %u byte", prefix);
    return false;
  }

  uint32_t sequence_millis;
  memcpy(&sequence_millis, this->rx_buffer, sizeof(sequence_millis));
  this->receive_packet.sequence_millis = ntohl(sequence_millis);

  memcpy(&this->receive_packet.type.receive, this->rx_buffer + 4,
         sizeof(this->receive_packet.type.receive));

  uint16_t size;
  memcpy(&size, this->rx_buffer + 5, sizeof(size));
  this->receive_packet.size = ntohs(size);

  log_d("Received packet. Millis %lu Type: %u Size: %u",
        this->receive_packet.sequence_millis, this->receive_packet.type.receive,
        this->receive_packet.size);

  uint8_t data =
      this->wifi_client.read(this->rx_buffer, this->receive_packet.size);
  if (data != this->receive_packet.size) {
    log_e("Unable to read full data, read %u byte", data);
    return false;
  }

  if (this->receive_packet.sequence_millis < this->rx_latest_millis) {
    log_d("Old packet received, discarding. %lu < %lu",
          this->receive_packet.sequence_millis, this->rx_latest_millis);
    return false;
  }

  this->rx_latest_millis = this->receive_packet.sequence_millis;
  return true;
}

void Protocol::AddSensor(Sensor* sensor) {
  this->sensors[this->sensors_count] = sensor;
  this->sensors_count++;
}

void Protocol::SendSensors() {
  size_t currentBufferOffset = 0;

  for (size_t i = 0; i < this->sensors_count; i++) {
    Sensor* sensor = this->sensors[i];
    size_t dataSize = sensor->getDataSize();

    if (dataSize > 0) {
      // Generate header for the sensor data
      int32_t headerSize = GenerateHeader(
          this->tx_buffer + currentBufferOffset, currentBufferOffset,
          RX_BUFFER_SIZE - currentBufferOffset, sensor->getMillis(),
          sensor->getPacketType(), dataSize);

      if (headerSize < 0) {
        log_e("Failed to generate header for sensor %d", i);
        continue;
      }

      // Serialize the sensor data after the header
      if (!sensor->serialize(
              this->tx_buffer,
              RX_BUFFER_SIZE - currentBufferOffset - headerSize)) {
        log_e("Failed to serialize sensor %d", i);
        continue;
      }

      currentBufferOffset += headerSize + dataSize;
    }
  }

  // Send all accumulated data in a single write if we have any
  if (currentBufferOffset > 0) {
    size_t bytesWritten =
        this->wifi_client.write(this->tx_buffer, currentBufferOffset);

    if (bytesWritten != currentBufferOffset) {
      log_e("Failed to send complete sensors packet. Expected: %d, Sent: %d",
            currentBufferOffset, bytesWritten);
    }
  }
}

int32_t Protocol::GenerateHeader(uint8_t* buffer, size_t start_index,
                                 size_t max_size, size_t millis,
                                 SendPacketType type, uint16_t size) {
  // Calculate the required size for the header
  const size_t header_size = Protocol::HeaderSize();

  // Check if there's enough space in the buffer
  if (start_index + header_size > max_size) {
    return -1;  // Not enough space
  }

  size_t offset = start_index;

  // Copy millis (4 bytes)
  memcpy(buffer + offset, &millis, sizeof(millis));
  offset += sizeof(millis);

  // Copy packet type (1 byte)
  memcpy(buffer + offset, &type, sizeof(type));
  offset += sizeof(type);

  // Copy size (2 bytes)
  memcpy(buffer + offset, &size, sizeof(size));
  offset += sizeof(size);

  return offset - start_index;  // Return number of bytes written
}
