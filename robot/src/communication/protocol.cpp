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
  /**
   * @brief Send the lidar buffer to the PC
   */

  // HomeRobotPacket packet = {};
  // packet.sequence_millis = this->buffer_start_millis;
  // packet.type.send = SendPacketType::TX_LIDAR;
  // packet.size = this->lidar_buffer_end;
  // packet.data = this->lidar_buffer;
  size_t offset = 0;

  // Coping the millis - 4 bytes
  memcpy(this->lidar_buffer, &this->buffer_start_millis, sizeof(this->buffer_start_millis));
  offset += sizeof(this->buffer_start_millis);

  // Coping the packet type - 1 byte
  const SendPacketType packet = TX_LIDAR;
  memcpy(this->lidar_buffer + offset, &packet, sizeof(packet));
  offset += sizeof(SendPacketType);

  // Coping the data size - 2 bytets
  this->lidar_buffer_end -= Protocol::PrefixSize();
  memcpy(this->lidar_buffer + offset, &this->lidar_buffer_end, sizeof(this->lidar_buffer_end));
  offset += sizeof(this->lidar_buffer_end);
  assert(offset == Protocol::PrefixSize());

  log_i("Sending lidar packet with size %d", this->lidar_buffer_end);
  size_t written = this->wifi_client.write(this->lidar_buffer, this->lidar_buffer_end + offset);
  if (written != this->lidar_buffer_end + offset) {
    sleep(2);
    written = this->wifi_client.write(this->lidar_buffer + written, this->lidar_buffer_end + offset - written);
  }
  assert(written == this->lidar_buffer_end + offset);

  this->lidar_buffer_end = PrefixSize();
  this->buffer_start_millis = 0;

}

void Protocol::AddLidarPacket(uint8_t* packet, uint16_t length) {
  if (this->lidar_buffer_end + length > MAX_LIDAR_BUFFER_SIZE) {
    log_i("Buffer is full, sending data");
    this->SendLidarPacket();
  }

  if (this->buffer_start_millis == 0) {
    this->buffer_start_millis = millis();
  }

  // Add the packet to the buffer
  memcpy(this->lidar_buffer + this->lidar_buffer_end, packet, length);
  this->lidar_buffer_end = this->lidar_buffer_end + length;
}

#include <arpa/inet.h>

bool Protocol::ReceivePacket() {
  uint8_t prefix = this->wifi_client.read(this->receive_buffer, Protocol::PrefixSize());
  if (prefix > 0 && prefix != Protocol::PrefixSize()) {
    log_e("Unable to read full prefix, read %u byte", prefix);
    return false;
  }

  uint32_t sequence_millis;
  memcpy(&sequence_millis, this->receive_buffer, sizeof(sequence_millis));
  this->receive_packet.sequence_millis = ntohl(sequence_millis);

  memcpy(&this->receive_packet.type.receive, this->receive_buffer + 4, sizeof(this->receive_packet.type.receive));

  uint16_t size;
  memcpy(&size, this->receive_buffer + 5, sizeof(size));
  this->receive_packet.size = ntohs(size);

  log_d("Received packet. Millis %lu Type: %u Size: %u",
    this->receive_packet.sequence_millis,
    this->receive_packet.type.receive,
    this->receive_packet.size);

  uint8_t data = this->wifi_client.read(this->receive_buffer, this->receive_packet.size);
  if (data != this->receive_packet.size) {
    log_e("Unable to read full data, read %u byte", data);
    return false;
  }

  if (this->receive_packet.sequence_millis < this->latest_rx_millis) {
    log_d("Old packet received, discarding. %lu < %lu",
          this->receive_packet.sequence_millis, this->latest_rx_millis);
    return false;
  }

  this->latest_rx_millis = this->receive_packet.sequence_millis;
  return true;
}

