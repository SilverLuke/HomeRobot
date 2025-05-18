#pragma once


#include <cstddef>  // for size_t
#include <cstdint>  // for uint8_t, uint16_t, int32_t
#include <Arduino.h>

#include "communication/packet_types.h"

class Sensor {
 public:
  virtual ~Sensor() = default;
  /**
   * @brief
   * @return String with name of the sensor.
   */
  virtual String name() = 0;
  /**
   * @brief The actual business logic for the sensor, read the data, update the
   * internal millis(), store the data in some internal buffer or stuff like
   * that.
   */
  virtual void read() = 0;

  /**
   * @brief This is used to build the message header when sending to the server.
   * @return The millis() of the latest sensor reading.
   */
  virtual uint32_t getMillis() = 0;
  /**
   * @brief
   * @return
   */
  virtual SendPacketType getPacketType() = 0;
  /**
   * @brief The data for the serialization, probably a useless function.
   * @return The internal data size in byte.
   */
  virtual uint16_t getDataSize() = 0;

  /**
   * @brief Serialize sensor data into the provided buffer
   * @param buffer Array where to write the serialized data
   * @param max_size Maximum available size in the buffer from the start_index
   * @return Number of bytes written to the buffer, or -1 if the buffer cannot
   * allocate all the serialization
   */
  virtual int32_t serialize(uint8_t* buffer, size_t max_size) = 0;
  /**
   * @brief This method will start the sensor, unlike the constructor, this
   * does not contain any initialization. E.G., The lidar will start the
   * motor. The IMU will do nothing or the calibration
   */
  virtual void startReading() {}
  /**
   * @brief This is the exact opposite of the startReading does. E.G., Lidar
   * will stop motor. IMU will do probably nothing
   */
  virtual void stopReading() {}

 protected:
  // Prevent instantiation of base class
  Sensor() = default;
};
