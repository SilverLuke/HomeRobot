cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(robot_app)

set(DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

list(APPEND CMAKE_MODULE_PATH ${ZEPHYR_BASE}/modules/nanopb)
include(nanopb)

target_sources(app PRIVATE 
    src/main.cpp
    src/robot.cpp
    src/communication/zephyr_net_client.cpp
    src/communication/wifi_manager.cpp
    src/communication/protobuf_handler.cpp
    src/actuator/motor.cpp
    src/actuator/status_led.cpp
    src/diagnostic.cpp
    src/sensors/lidar.cpp
    src/sensors/imu.cpp
    src/sensors/battery.cpp
    src/sensors/encoders.cpp
    src/sensors/pcnt_reader.c
)

if(CONFIG_BOARD_NATIVE_SIM)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GZ_TRANSPORT REQUIRED IMPORTED_TARGET gz-transport)
    pkg_check_modules(GZ_MSGS REQUIRED IMPORTED_TARGET gz-msgs)
    
    # Get Protobuf include directory explicitly to force it first for Gazebo compatibility
    pkg_check_modules(GZ_PROTOBUF REQUIRED protobuf)
    target_include_directories(app BEFORE PRIVATE ${GZ_PROTOBUF_INCLUDE_DIRS})
    
    target_sources(app PRIVATE src/bridge/gazebo_bridge.cpp)
    
    # Provide include directories for Gazebo headers
    target_include_directories(app PRIVATE 
        ${GZ_TRANSPORT_INCLUDE_DIRS} 
        ${GZ_MSGS_INCLUDE_DIRS}
    )

    # Enable C++17 for Gazebo headers
    target_compile_features(app PRIVATE cxx_std_17)
    
    # Prevent Protobuf version check errors
    target_compile_definitions(app PRIVATE 
        PROTOBUF_USE_DLLS
    )
    
    # Link Gazebo libraries using absolute paths for the native_simulator Makefile
    foreach(lib ${GZ_TRANSPORT_LIBRARIES} ${GZ_MSGS_LIBRARIES})
        find_library(LIB_PATH_${lib} NAMES ${lib} HINTS ${GZ_TRANSPORT_LIBRARY_DIRS} ${GZ_MSGS_LIBRARY_DIRS})
        if(LIB_PATH_${lib})
            set_property(TARGET native_simulator APPEND PROPERTY RUNNER_LINK_LIBRARIES ${LIB_PATH_${lib}})
        endif()
    endforeach()

    # Also link Protobuf and its dependencies explicitly as they are required by Gazebo
    foreach(lib ${GZ_PROTOBUF_LIBRARIES})
        find_library(LIB_PATH_${lib} NAMES ${lib} HINTS ${GZ_PROTOBUF_LIBRARY_DIRS})
        if(LIB_PATH_${lib})
            set_property(TARGET native_simulator APPEND PROPERTY RUNNER_LINK_LIBRARIES ${LIB_PATH_${lib}})
        endif()
    endforeach()
endif()

zephyr_nanopb_sources(app ../proto/messages.proto)

target_include_directories(app PRIVATE src)
zephyr_include_directories(include)
add_subdirectory(drivers)

target_compile_options(app PRIVATE -fno-threadsafe-statics -fpermissive)
