#include <Arduino.h>
#include <arpa/inet.h>
#include <errno.h>
#include <gtest/gtest.h>
#include <sys/unistd.h>

#include <cstdint>
#include <cstring>

#include "../../src/communication/protocol.h"
#include "../../src/communication/net_client.h"
#include "../../src/communication/packet_types.h"

const uint8_t HEADER_SIZE = 7;

class FakeClient : public NetClient {
public:
  uint8_t* input;
  size_t input_size;

  uint8_t* chunks;
  size_t chunks_size;

  size_t in_pos = 0;
  size_t chunk_pos = 0;
  bool connected_flag = true;
  uint8_t* output;

  void setInput(
    uint8_t* data,
    size_t data_length,
    uint8_t* per_read,
    size_t per_read_size
  ) {
    input = data;
    input_size = data_length;
    chunks = per_read;
    chunks_size = per_read_size;
    in_pos = 0;
    chunk_pos = 0;
  }

  // NetClient impl
  std::size_t read(uint8_t* buffer, std::size_t size) override {
    if (in_pos >= input_size) return 0;
    size_t remaining = input_size - in_pos;
    size_t to_read = std::min<size_t>(size, remaining);
    if (chunk_pos < chunks_size) {
      to_read = std::min<size_t>(to_read, chunks[chunk_pos]);
      chunk_pos++;
    }
    if (to_read == 0) return 0;
    memcpy(buffer, input + in_pos, to_read);
    in_pos += to_read;
    return to_read;
  }

  std::size_t write(const uint8_t* buffer, std::size_t size) override {
    return 0;
    // output.insert(output.end(), buffer, buffer + size);
    // return size;
  }

  bool connected() override { return connected_flag; }
  bool connect(const char*, uint16_t) override { connected_flag = true; return true; }
  void stop() override { connected_flag = false; }
  void flush() override {}
};

static uint8_t* make_packet(uint32_t millis, ReceivePacketType type, uint16_t payload_size, const uint8_t* payload) {
  uint8_t* v = (uint8_t*) malloc(payload_size + HEADER_SIZE);
  uint32_t m_be = htonl(millis);
  memcpy(v, &m_be, 4);
  v[4] = static_cast<uint8_t>(type);
  uint16_t s_be = htons(payload_size);
  memcpy(v + 5, &s_be, 2);
  if (payload_size > 0) {
    memcpy(v + HEADER_SIZE, payload, payload_size);
  }
  return v;
}