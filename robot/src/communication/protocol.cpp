#include "protocol.h"
#include "secrets.h"
#include <WiFi.h>
#include <cstdint>


Protocol::Protocol() {
  // Initialize the Wi-Fi client
  if (WiFi.isConnected()) {
    this->wifi_client = WiFiClient();
    this->connect();
  }

  this->receive_packet.data = this->receive_buffer;
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
    while (! this->wifi_client.connect(wifi_server_host, wifi_server_port) && attempts < 5) {
      Serial.println("Failed to connect to server");
      attempts++;
      delay(1000);
    }
    Serial.println("Connected to server");
}

bool Protocol::is_connected() {
  return this->wifi_client.connected();
}

void Protocol::restart() {
  if (this->wifi_client.connected()) {
    this->wifi_client.stop();
  }
  connect();
  this->receive_packet = {};
  this->buffer_start_millis = 0;
  this->lidar_buffer_end = Protocol::PrefixSize();
}

size_t Protocol::PrefixSize() {
    return sizeof(HomeRobotPacket::sequence_millis) + sizeof(HomeRobotPacket::type) + sizeof(HomeRobotPacket::size);
}

size_t Protocol::SendPacket(const HomeRobotPacket& packet) {
    if (this->wifi_client.connected()) {
      const size_t constant_offset = Protocol::PrefixSize();

      const size_t totalSize = constant_offset + packet.size;

      memcpy(this->packet_buffer , &packet, constant_offset);

      // Copy the data array if it exists
      if (packet.data && packet.size > 0) {
          memcpy(this->packet_buffer + constant_offset, packet.data, packet.size);
      }

      // Send data
      const size_t bytesWritten = this->wifi_client.write(this->packet_buffer, totalSize);
      if ( bytesWritten != totalSize) {
        log_e("Error writing packet");
      } else {
        // Serial.println("Data sent");
      }
      return bytesWritten;
    }
    else {
      log_e("Error client not connected");
    }
    return 0;
}

void Protocol::SendLidarPacket() {
  // HomeRobotPacket packet = {};
  // packet.sequence_millis = this->buffer_start_millis;
  // packet.type.send = SendPacketType::TX_LIDAR;
  // packet.size = this->lidar_buffer_end;
  // packet.data = this->lidar_buffer;
  size_t offset = 0;

  memcpy(this->lidar_buffer, &this->buffer_start_millis, sizeof(this->buffer_start_millis));
  offset += sizeof(this->buffer_start_millis);

  this->lidar_buffer[offset] = SendPacketType::TX_LIDAR;
  offset += sizeof(SendPacketType);

  memcpy(this->lidar_buffer + offset, &this->lidar_buffer_end, sizeof(this->lidar_buffer_end));
  offset += sizeof(this->lidar_buffer_end);
  assert(offset == Protocol::PrefixSize());

  size_t written = this->wifi_client.write(this->lidar_buffer,  offset + this->lidar_buffer_end);
  assert(written == offset + this->lidar_buffer_end);

  this->lidar_buffer_end = Protocol::PrefixSize();
  this->buffer_start_millis = 0;

}

void Protocol::AddLidarPacket(uint8_t* packet, uint16_t length) {
  if (this->lidar_buffer_end + length > MAX_LIDAR_BUFFER_SIZE) {
    log_d("Buffer is full, sending data");
    this->SendLidarPacket();
  }

  if (this->buffer_start_millis == 0) {
    this->buffer_start_millis = millis();
  }

  memcpy(this->lidar_buffer + this->lidar_buffer_end, packet, length);
  this->lidar_buffer_end = this->lidar_buffer_end + length;
}


bool Protocol::ReceivePacket() {
  uint8_t prefix = this->wifi_client.read(this->receive_buffer, Protocol::PrefixSize());
  if (prefix > 0 && prefix != Protocol::PrefixSize()) {
    log_e("Unable to read full prefix");
    return false;
  }

  uint8_t data = this->wifi_client.read(this->receive_buffer, this->receive_packet.size);
  if (data != this->receive_packet.size) {
    log_e("Unable to read full data");
    return false;
  }

  return true;
}
