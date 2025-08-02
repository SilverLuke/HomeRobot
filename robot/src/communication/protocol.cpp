#include "protocol.h"

#include <Elog.h>
#include <Esp.h>
#include <WiFi.h>
#include <arpa/inet.h>
#include <sys/unistd.h>

#include <cstdint>

#include "definitions.h"
#include "packet_types.h"
#include "secrets.h"
#include "utils/utils.h"

Protocol::Protocol() {
  // Initialize the Wi-Fi client
  if (WiFi.isConnected()) {
    this->server_connection = WiFiClient();
    this->connect();
  }
}

void Protocol::connect() {
  int attempts = 0;
  while (!this->server_connection.connect(wifi_server_host, wifi_server_port) &&
         attempts++ < 5) {
    Logger.debug(PROTO_LOGGER, "Failed to connect to server");
    delay(2000);
  }
  if (attempts >= 5) {
    Logger.error(PROTO_LOGGER, "Failed to connect to server");
  } else {
    Logger.info(PROTO_LOGGER, "Connected to server");
  }
}

bool Protocol::isConnected() { return this->server_connection.connected(); }

/**
 * @brief This will reset the Wi-Fi and the server connection
 */
void Protocol::HardRestart() {
  restart_wifi();
  this->SoftRestart();
  this->connect();
}

/**
 * @brief This will reset the connection to the server and reset the
 * variables in the protocol
 */
void Protocol::SoftRestart() {
  this->receive_packet = {};
  this->rx_latest_millis = 0;
  this->rx_used = 0;
  this->read_header = false;

  if (!this->server_connection.connected()) {
    Logger.info(PROTO_LOGGER, "Server not connected, restarting");
    this->server_connection = WiFiClient();
    this->connect();
  }
}

void Protocol::ShowStatus() {
    Logger.info(PROTO_LOGGER, "Wifi %d, Server: %d, Latest millis: %lu, Used: %d, Read header: %d, Sensors count: %d",
              WiFi.isConnected(),
              this->server_connection.connected(),
              this->rx_latest_millis,
              this->rx_used,
              this->read_header,
              this->sensors_count
              );
  if (this->server_connection.connected()) {
    neopixelWrite(RGB_BUILTIN, LED_GREEN);
  } else if (WiFi.isConnected()) {
    neopixelWrite(RGB_BUILTIN, LED_ORANGE);
  } else {
    neopixelWrite(RGB_BUILTIN, LED_RED);
  }
}

size_t Protocol::HeaderSize() {
  return sizeof(HomeRobotHeader);
}

size_t Protocol::SendPacket(const HomeRobotPacket& packet) {
  if (this->server_connection.connected()) {
    const size_t constant_offset = Protocol::HeaderSize();

    const size_t totalSize = constant_offset + packet.header.size;

    memcpy(this->tx_buffer, &packet, constant_offset);

    // Copy the data array if it exists
    if (packet.data && packet.header.size > 0) {
      memcpy(this->tx_buffer + constant_offset, packet.data, packet.header.size);
    }

    // Send data
    const size_t bytesWritten =
        this->server_connection.write(this->tx_buffer, totalSize);
    if (bytesWritten != totalSize) {
      Logger.error(PROTO_LOGGER, "Error writing packet");
    } else {
      // Logger.debug(PROTO_LOGGER, "Data sent");
    }
    return bytesWritten;
  } else {
    Logger.error(PROTO_LOGGER, "Error client not connected");
  }
  return 0;
}


void Protocol::ParseHeader() {
  uint32_t sequence_millis;
  memcpy(&sequence_millis, this->rx_buffer, sizeof(sequence_millis));
  this->receive_packet.header.sequence_millis = ntohl(sequence_millis);

  memcpy(&this->receive_packet.header.type.receive, this->rx_buffer + sizeof(uint32_t),
         sizeof(this->receive_packet.header.type.receive));
  uint16_t size;
  memcpy(&size, this->rx_buffer + 5, sizeof(size));
  this->receive_packet.header.size = ntohs(size);

  Logger.debug(PROTO_LOGGER, "Received packet. Millis %lu Type: %u Size: %u",
        this->receive_packet.header.sequence_millis, this->receive_packet.header.type.receive,
        this->receive_packet.header.size);
  // Now it is ready to read
  this->read_header = true;
}

#include <errno.h>

bool Protocol::ReceivePacket() {
  errno = 0;
  // Read the TCP channel from wifi.
  int16_t prefix = this->server_connection.read(
    this->rx_buffer + this->rx_used,
    RX_BUFFER_SIZE - this->rx_used
  );
  if (errno > 0) {
    Logger.error(PROTO_LOGGER, "Error reading from server: %s", strerror(errno));
    return false;
  }
  if (prefix < 0) {
    Logger.error(PROTO_LOGGER, "Unable to read %d", prefix);
    return false;
  }

  this->rx_used += prefix;
  // Not enough data to parse the header
  if (this->rx_used < HeaderSize()) {
    return false;
  }

  // If header is read skip this part
  if (this->read_header == false) {
    this->ParseHeader();
  }

  size_t total_size = HeaderSize() + this->receive_packet.header.size;
  if (total_size > RX_MAX_DATA) {
    Logger.error(PROTO_LOGGER, "Received packet too large. Size: %d", total_size);
    // Reset buffer and header state
    this->rx_used = 0;
    this->read_header = false;
    return false;
  }
  
  if (this->rx_used >= total_size) {
    // Copy data FROM buffer TO packet (corrected direction)
    if (this->receive_packet.header.size > 0) {
      memcpy(this->receive_packet.data, this->rx_buffer + HeaderSize(), this->receive_packet.header.size);
    }

    // Copy the next packet at the beginning of the buffer, for the next cycle.
    this->rx_used = this->rx_used - total_size;
    memmove(this->rx_buffer, this->rx_buffer + total_size, this->rx_used);
  } else {
    return false;
  }

  // Check if the current packet is newer than the latest (error in transmission or in the server)
  if (this->receive_packet.header.sequence_millis < this->rx_latest_millis) {
    Logger.debug(PROTO_LOGGER, "Old packet received, discarding. %lu < %lu",
          this->receive_packet.header.sequence_millis, this->rx_latest_millis);
    return false;
  }

  this->read_header = false;
  this->rx_latest_millis = this->receive_packet.header.sequence_millis;
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
    uint16_t dataSize = sensor->getDataSize();

    if (dataSize > 0) {
      Logger.debug(PROTO_LOGGER, "Creating packet for sensor %s with size %d", sensor->name(), dataSize);
      // Generate header for the sensor data
      int32_t headerSize = GenerateHeader(
          this->tx_buffer + currentBufferOffset,
          TX_BUFFER_SIZE - currentBufferOffset,
          sensor->getMillis(),
          sensor->getPacketType(),
          dataSize
      );

      if (headerSize < 0 || headerSize != 7) {
        Logger.error(PROTO_LOGGER, "Failed to generate header for sensor %s. Header size %d", sensor->name(), headerSize);
        continue;
      }
      currentBufferOffset += headerSize;

      // Serialize the sensor data after the header
      int32_t serialization = sensor->serialize(
              this->tx_buffer + currentBufferOffset,
              TX_BUFFER_SIZE - currentBufferOffset
              );
      if (serialization < 0 || serialization != dataSize) {
        Logger.error(PROTO_LOGGER, "Failed to serialize sensor %s", sensor[i].name());
        continue;
      }
      // Todo handle roll back if a sensor was unable to serialize.

      currentBufferOffset += serialization;
      Logger.debug(PROTO_LOGGER, "Current buffer offset %d", currentBufferOffset);
    }
  }

  // Send all accumulated data in a single write if we have any
  if (currentBufferOffset > 0) {
    Logger.info(PROTO_LOGGER, "Sending %d bytes of sensors data", currentBufferOffset);
    // Logger.info(PROTO_LOGGER, "Memory - Total: %d, Free: %d, Used: %d",
    //       ESP.getHeapSize(),
    //       ESP.getFreeHeap(),
    //       ESP.getHeapSize() - ESP.getFreeHeap());

    size_t bytesWritten =
        this->server_connection.write(this->tx_buffer, currentBufferOffset);
    if (bytesWritten == 0) {
      Logger.error(PROTO_LOGGER, "Failed to send complete sensors packet. Expected: %d, Sent: %d", currentBufferOffset, bytesWritten);
      this->server_connection.stop();
      this->HardRestart();
    } else {
      this->server_connection.flush();
    }
  }
}

void Protocol::Loop() {
  for (size_t i = 0; i < this->sensors_count; i++) {
    this->sensors[i]->read();
  }
  if (this->server_connection.connected()) {
    this->SendSensors();
  }
}

int32_t Protocol::GenerateHeader(
  uint8_t* buffer,
  size_t buffer_size,
  uint32_t millis,
  SendPacketType type,
  uint16_t size
) {
  // Calculate the required size for the header
  Logger.info(PROTO_LOGGER, "Generating header millis %d type %d size %d", millis, type, size);
  const size_t header_size = Protocol::HeaderSize();

  // Check if there's enough space in the buffer
  if (header_size > buffer_size) {
    return -1;  // Not enough space
  }

  size_t offset = 0;

  // Copy millis (4 bytes)
  uint32_t millis_ntohl = htonl(millis);
  memcpy(buffer, &millis_ntohl, sizeof(millis));
  offset += sizeof(millis);

  // Copy packet type (1 byte)
  memcpy(buffer + offset, &type, sizeof(type));
  offset += sizeof(type);

  // Copy size (2 bytes)
  uint16_t size_ntohs = htons(size);
  memcpy(buffer + offset, &size_ntohs, sizeof(size));
  offset += sizeof(size);

  return offset;  // Return number of bytes written
}