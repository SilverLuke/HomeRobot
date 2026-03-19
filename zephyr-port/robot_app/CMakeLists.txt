cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(robot_app)

list(APPEND CMAKE_MODULE_PATH ${ZEPHYR_BASE}/modules/nanopb)
include(nanopb)

target_sources(app PRIVATE 
    src/main.cpp
    src/communication/zephyr_net_client.cpp
    src/communication/wifi_manager.cpp
    src/communication/protobuf_handler.cpp
    src/actuator/motor.cpp
    src/sensors/lidar.cpp
    src/sensors/imu.cpp
    src/sensors/battery.cpp
    src/sensors/encoders.cpp
)

zephyr_nanopb_sources(app ../../proto/messages.proto)

target_include_directories(app PRIVATE src)

target_compile_options(app PRIVATE -fno-threadsafe-statics)
