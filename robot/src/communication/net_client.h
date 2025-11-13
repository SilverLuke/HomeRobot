#pragma once

#include <cstddef>
#include <cstdint>

// Minimal network client interface to enable testing Protocol without Arduino WiFiClient
class NetClient {
public:
  virtual ~NetClient() = default;

  // Read up to size bytes into buffer, return number of bytes read.
  virtual std::size_t read(uint8_t* buffer, std::size_t size) = 0;

  // Write size bytes from buffer, return number of bytes written.
  virtual std::size_t write(const uint8_t* buffer, std::size_t size) = 0;

  // Is the client connected
  virtual bool connected() = 0;

  // Connect to host:port (may be no-op for adapters already connected)
  virtual bool connect(const char* host, uint16_t port) = 0;

  // Stop/close the connection
  virtual void stop() = 0;

  // Flush any pending outgoing bytes (no-op for some implementations)
  virtual void flush() = 0;
};
