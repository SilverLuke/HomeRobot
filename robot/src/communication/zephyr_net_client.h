#pragma once

#include "net_client.h"

#include <zephyr/net/socket.h>

class ZephyrNetClient : public NetClient {
public:
  ZephyrNetClient();
  ~ZephyrNetClient() override;

  std::size_t read(uint8_t* buffer, std::size_t size) override;
  std::size_t write(const uint8_t* buffer, std::size_t size) override;
  bool connected() override;
  bool connect(const char* host, uint16_t port) override;
  void stop() override;
  void flush() override;

private:
  int sock_fd_ = -1;
  bool connected_ = false;
};
