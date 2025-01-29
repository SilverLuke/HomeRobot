#include "protocol.h"
#include "secrets.h"
#include <WiFi.h>
#include <cstdint>


void connect(WiFiClient& client) {
    int attempts = 0;
    while (! client.connect(wifi_server_host, wifi_server_port) && attempts < 5) {
      Serial.println("Failed to connect to server");
      attempts++;
      delay(1000);
    }
    Serial.println("Connected to server");
}

Protocol::Protocol() {
    // Initialize the WiFi client
    this->client = WiFiClient();
    connect(this->client);

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
}

void Protocol::restart() {
    if (this->client.connected()) {
        this->client.stop();
    }
    connect(this->client);
}

size_t Protocol::writePacket(const HomeRobotPacket& packet) {
    if (this->client.connected()) {
      // Calculate the total size of the packet, including the data array

      size_t constant_offset = sizeof(packet.millis) + sizeof(packet.type) + sizeof(packet.size);



      size_t totalSize = constant_offset + packet.size;
      // Create a buffer to hold the entire struct (including the data)

      // Copy the fixed part of the struct (everything except the data array)
      memcpy(wifi_buffer, &packet, constant_offset);

      // Copy the data array if it exists
      if (packet.data && packet.size > 0) {
          memcpy(wifi_buffer + constant_offset, packet.data, packet.size);
      }

      // Send data
      size_t bytesWritten = this->client.write(wifi_buffer, totalSize);
      if ( bytesWritten != totalSize) {
        Serial.println("Error writing packet");
      } else {
        // Serial.println("Data sent");
      }
      return bytesWritten;
    }
    else {
      Serial.println("Error client not connected");
    }
    return 0;
}
