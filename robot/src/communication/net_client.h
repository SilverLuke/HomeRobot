#pragma once

#include <cstddef>
#include <cstdint>

/**
 * @brief Minimal network client interface.
 * 
 * This interface is used to abstract the network communication, 
 * allowing to use different implementations (e.g. Arduino WiFiClient, Zephyr BSD Sockets).
 */
class NetClient {
public:
  virtual ~NetClient() = default;

  /**
   * @brief Read up to size bytes into buffer.
   * @return number of bytes read.
   */
  virtual std::size_t read(uint8_t* buffer, std::size_t size) = 0;

  /**
   * @brief Write size bytes from buffer.
   * @return number of bytes written.
   */
  virtual std::size_t write(const uint8_t* buffer, std::size_t size) = 0;

  /**
   * @brief Is the client connected.
   */
  virtual bool connected() = 0;

  /**
   * @brief Connect to host:port.
   */
  virtual bool connect(const char* host, uint16_t port) = 0;

  /**
   * @brief Stop/close the connection.
   */
  virtual void stop() = 0;

  /**
   * @brief Flush any pending outgoing bytes.
   */
  virtual void flush() = 0;
};
