#pragma once

#include <zephyr/kernel.h>

namespace constants {
    // Telemetry Intervals (milliseconds)
    constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000;
    constexpr uint32_t ENCODER_TELEMETRY_INTERVAL_MS = 1000;
    constexpr uint32_t BATTERY_TELEMETRY_INTERVAL_MS = 30000;
    
    // Lidar Configuration
    constexpr size_t LIDAR_BATCH_SIZE = 30;
    constexpr float LIDAR_DEFAULT_FREQ_HZ = 5.0f;

    // Networking
    constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 30000;
    constexpr uint32_t SERVER_RECONNECT_INTERVAL_MS = 5000;

    // Main Loop
    constexpr uint32_t MAIN_LOOP_DELAY_MS = 10; // 100Hz loop
}
