#pragma once

#include <WiFi.h>
#include "communication/net_client.h"

class WiFiClientAdapter : public NetClient {
public:
  WiFiClientAdapter() = default;
  ~WiFiClientAdapter() override = default;

  std::size_t read(uint8_t* buffer, std::size_t size) override {
    return static_cast<std::size_t>(client.read(buffer, size));
  }

  std::size_t write(const uint8_t* buffer, std::size_t size) override {
    return static_cast<std::size_t>(client.write(buffer, size));
  }

  bool connected() override { return client.connected(); }

  bool connect(const char* host, uint16_t port) override { return client.connect(host, port); }

  void stop() override { client.stop(); }

  void flush() override { client.flush(); }

private:
  WiFiClient client;
};
