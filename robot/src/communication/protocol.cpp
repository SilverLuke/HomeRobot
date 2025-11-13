#include "protocol.h"

#include <Arduino.h>  // Import required to fix some import error
#include <Elog.h>
#include <WiFi.h>  // Import required to fix some import error
#include <arpa/inet.h>
#include <sys/unistd.h>

#include <cerrno>
#include <cstdint>

#include "definitions.h"
#include "packet_types.h"
#include "secrets.h"
#include "utils/utils.h"
#include "wifi_client_adapter.h"

Protocol::Protocol(NetClient* client) {
  this->sensors_count = 0;
  this->stats = new ProtocolStats();
  this->server_connection = client;
  this->receive_packet = {};
  this->connect();
}

Protocol::Protocol() : Protocol(new WiFiClientAdapter) {
}

void Protocol::connect() const {
  int attempts = 0;
  while (this->server_connection && !this->server_connection->connect(wifi_server_host, wifi_server_port) &&
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

bool Protocol::isConnected() const {
  return this->server_connection && this->server_connection->connected();
}

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
  this->rx_buffer_index = 0;
  this->read_header = false;

  this->stats = new ProtocolStats();

  if (!(this->server_connection && this->server_connection->connected())) {
    Logger.info(PROTO_LOGGER, "Server not connected, restarting");
    // Recreate adapter
    delete this->server_connection;
    this->server_connection = new WiFiClientAdapter();
    this->connect();
  }
}

void Protocol::ShowStatus() {
  Logger.info(PROTO_LOGGER, "Wifi %d, Server: %d, Latest millis: %lu, Used: %d, Read header: %d, Sensors count: %d",
    WiFi.isConnected(),
    this->server_connection && this->server_connection->connected(),
    this->rx_latest_millis,
    this->rx_buffer_index,
    this->read_header,
    this->sensors_count
  );
  if (this->server_connection && this->server_connection->connected()) {
    neopixelWrite(RGB_BUILTIN, LED_GREEN);
  } else if (WiFi.isConnected()) {
    neopixelWrite(RGB_BUILTIN, LED_ORANGE);
  } else {
    neopixelWrite(RGB_BUILTIN, LED_RED);
  }
}

/**
 * @brief
 * @return The size in bytes of the header. It should be 10, 4 timestamp, 1
 * type, 2 length
 */
size_t Protocol::HeaderSize() {
  // Network header is fixed: 4 bytes millis, 1 byte type, 2 bytes size
  return sizeof(HomeRobotHeader);
}

size_t Protocol::SendPacket(const HomeRobotPacket& packet) {
  if (this->server_connection && this->server_connection->connected()) {
    const size_t constant_offset = Protocol::HeaderSize();

    const size_t totalSize = constant_offset + packet.header.size;

    memcpy(this->tx_buffer, &packet, constant_offset);

    // Copy the data array if it exists
    if (packet.data && packet.header.size > 0) {
      memcpy(this->tx_buffer + constant_offset, packet.data, packet.header.size);
    }

    // Send data
    const size_t bytesWritten =
        this->server_connection->write(this->tx_buffer, totalSize);
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

  Logger.debug(PROTO_LOGGER,
    "Parsed header. Millis %lu Type: %u Size: %u",
        this->receive_packet.header.sequence_millis,
        this->receive_packet.header.type.receive,
        this->receive_packet.header.size
        );
  // Now it is ready to read
  this->read_header = true;
}

bool Protocol::process_next_packet() {
  // Not enough data to parse the header
  if (this->rx_buffer_index < HeaderSize()) {
    return false;
  }

  // If header is read skip this part
  if (this->read_header == false) {
    this->ParseHeader();
    this->stats->rx_packets++;
  }

  const size_t total_packet_size = HeaderSize() + this->receive_packet.header.size;
  // Packet not fully read
  if (total_packet_size < this->rx_buffer_index) {
    return false;
  }

  if (total_packet_size > RX_MAX_PACKET_SIZE) {
    Logger.error(PROTO_LOGGER, "Received packet too large. Size: %d",
                 total_packet_size);
    // Reset buffer and header state. This can cause sync issues between the cloud and the server.
    this->rx_buffer_index = 0;
    this->read_header = false;
    return false;
  }

  if (this->rx_buffer_index >= total_packet_size) {
    // Copy data FROM buffer TO packet (corrected direction)
    if (this->receive_packet.header.size > 0) {
      memcpy(
        this->receive_packet.data,
        this->rx_buffer + HeaderSize(),
        this->receive_packet.header.size
      );
    }

    // Copy the next packet at the beginning of the buffer, for the next cycle.
    // TODO improve this with ring buffer
    const size_t to_move = this->rx_buffer_index - total_packet_size;
    memmove(
      this->rx_buffer,
      this->rx_buffer + total_packet_size,
      to_move
    );
    this->rx_buffer_index = to_move;
  }
  else {
    return false;
  }

  // Check if the current packet is newer than the latest (error in transmission
  // or in the server)
  if (this->receive_packet.header.sequence_millis < this->rx_latest_millis) {
    Logger.debug(
      PROTO_LOGGER,
      "Old packet received, discarding. %lu < %lu",
      this->receive_packet.header.sequence_millis,
      this->rx_latest_millis
    );
    return false;
  }

  this->read_header = false;
  this->rx_latest_millis = this->receive_packet.header.sequence_millis;
  return true;
}

void Protocol::read_raw_data() {
  const size_t space_available = RX_BUFFER_SIZE - this->rx_buffer_index;
  if (space_available == 0) {
    Logger.debug(PROTO_LOGGER, "No space left for new packet!");
    return;
  }

  errno = 0;

  // Read the TCP channel from wifi.
  uint16_t read_bytes = 0;
  read_bytes = this->server_connection->read(
      this->rx_buffer + this->rx_buffer_index,
      space_available
      );

  if (errno != 0) {
    Logger.error(
      PROTO_LOGGER,
      "Error reading from server: %d %s",
      strerror(errno),
      errno
    );
    return;
  }

  this->rx_buffer_index += read_bytes;
  this->stats->rx_bytes += read_bytes;
}

bool Protocol::ReceivePacket() {
  this->read_raw_data();
  return this->process_next_packet();
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
    const char* sensor_name = sensor->name().c_str();

    if (dataSize > 0) {
      Logger.debug(PROTO_LOGGER, "Creating packet for sensor %s with size %d", sensor_name, dataSize);
      // Generate header for the sensor data
      int32_t headerSize = GenerateHeader(
          this->tx_buffer + currentBufferOffset,
          TX_BUFFER_SIZE - currentBufferOffset,
          sensor->getMillis(),
          sensor->getPacketType(),
          dataSize
      );

      if (headerSize < 0 || headerSize != 7) {
        Logger.error(PROTO_LOGGER, "Failed to generate header for sensor %s. Header size %d", sensor_name, headerSize);
        continue;
      }
      currentBufferOffset += headerSize;

      // Serialize the sensor data after the header
      int32_t serialization = sensor->serialize(
        this->tx_buffer + currentBufferOffset,
        TX_BUFFER_SIZE - currentBufferOffset
        );

      if (serialization < 0 || serialization != dataSize) {
        Logger.error(PROTO_LOGGER, "Failed to serialize sensor %s", sensor_name);
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
        this->server_connection->write(this->tx_buffer, currentBufferOffset);

    if (bytesWritten == 0) {
      Logger.error(PROTO_LOGGER, "Failed to send complete sensors packet. Expected: %d, Sent: %d", currentBufferOffset, bytesWritten);
      this->server_connection->stop();
      this->HardRestart();
    } else {
      if (this->stats) this->stats->tx_bytes += bytesWritten;
      this->server_connection->flush();
    }
  }
}

void Protocol::Loop() {
  for (size_t i = 0; i < this->sensors_count; i++) {
    this->sensors[i]->read();
  }
  if (this->server_connection && this->server_connection->connected()) {
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