# CMakeLists.txt for Zephyr LED Application
# Updated to support both ESP32 C6 and S3 DevKit boards

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephyr_led_app)

# Add source files
target_sources(app PRIVATE src/main.c)

# Add custom drivers and includes
zephyr_include_directories(include)
add_subdirectory(drivers)
