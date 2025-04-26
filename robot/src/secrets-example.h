#pragma once
#include <IPAddress.h>

#include <cstdint>

inline const char* wifi_ssid = "WiFi name";
inline const char* wifi_password = "WiFi password";

// Server details
inline const char* wifi_server_host = "192.168.1.1";
inline constexpr uint16_t wifi_server_port = 12345;

// Set your desired IP address
inline IPAddress local_IP(192, 168, 199, 123);
inline IPAddress gateway(192, 168, 199, 254);
inline IPAddress subnet(255, 255, 255, 0);
