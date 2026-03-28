#include "zephyr_net_client.h"

#include <errno.h>

#ifdef CONFIG_NETWORKING
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ZephyrNetClient, LOG_LEVEL_DBG);
#endif

ZephyrNetClient::ZephyrNetClient() {
}

ZephyrNetClient::~ZephyrNetClient() {
  stop();
}

std::size_t ZephyrNetClient::read(uint8_t* buffer, std::size_t size) {
#ifdef CONFIG_NETWORKING
  if (sock_fd_ < 0 || !connected_) {
    return 0;
  }

  // Use non-blocking read for now
  ssize_t bytes_read = zsock_recv(sock_fd_, buffer, size, ZSOCK_MSG_DONTWAIT);
  if (bytes_read < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;
    }
    LOG_ERR( "Recv error: %d", errno);
    stop();
    return 0;
  } else if (bytes_read == 0) {
    LOG_INF( "Connection closed by peer");
    stop();
    return 0;
  }

  return static_cast<std::size_t>(bytes_read);
#else
  return 0;
#endif
}

std::size_t ZephyrNetClient::write(const uint8_t* buffer, std::size_t size) {
#ifdef CONFIG_NETWORKING
  if (sock_fd_ < 0 || !connected_) {
    return 0;
  }

  ssize_t bytes_written = zsock_send(sock_fd_, buffer, size, 0);
  if (bytes_written < 0) {
    LOG_ERR( "Send error: %d", errno);
    stop();
    return 0;
  }

  return static_cast<std::size_t>(bytes_written);
#else
  return 0;
#endif
}

bool ZephyrNetClient::connected() {
  return connected_ && sock_fd_ >= 0;
}

bool ZephyrNetClient::connect(const char* host, uint16_t port) {
#ifdef CONFIG_NETWORKING
  if (connected_) {
    stop();
  }

  sock_fd_ = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock_fd_ < 0) {
    LOG_ERR( "Could not create socket: %d", errno);
    return false;
  }

  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);

  if (zsock_inet_pton(AF_INET, host, &addr.sin_addr) <= 0) {
    LOG_ERR( "Invalid host IP: %s", host);
    zsock_close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }

  LOG_INF( "Connecting to %s:%d...", host, port);
  if (zsock_connect(sock_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    LOG_ERR( "Connection failed: %d", errno);
    zsock_close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }

  connected_ = true;
  LOG_INF( "Connected successfully");
  return true;
#else
  return false;
#endif
}

void ZephyrNetClient::stop() {
#ifdef CONFIG_NETWORKING
  if (sock_fd_ >= 0) {
    zsock_close(sock_fd_);
    sock_fd_ = -1;
  }
#endif
  connected_ = false;
}

void ZephyrNetClient::flush() {
  // No-op for Zephyr BSD sockets (write already sends)
}
