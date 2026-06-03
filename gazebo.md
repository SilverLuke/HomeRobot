Implementing a Zephyr to Gazebo Bridge is a highly effective way to accelerate development by allowing you to test
your RTOS logic on your Linux machine before deploying to the ESP32-C6.

Based on your current robot_app structure, here is the technical blueprint and implementation strategy.

1. Architecture: The "UDP Shim" Pattern
   To keep your code clean, we will use a Virtual Driver pattern. The high-level Motor and Encoders classes will
   remain the same, but their implementation will switch between Hardware (PWM/PCNT) and Simulation (UDP) at
   compile-time.

---

2. Implementation Blueprint (LLM Input)

System Prompt: Act as an Embedded Systems Engineer specializing in Zephyr RTOS and Robotics.

Task: Generate a project extension for homeRobot that adds native_sim support to bridge Zephyr with Gazebo.

Requirements:

A. Zephyr Side: native_sim Configuration

1.  Overlay Config (boards/native_sim.conf):

1 # Enable networking for Linux host
2 CONFIG\*NET_SOCKETS=y
3 CONFIG_NET_UDP=y
4 CONFIG_NET_IPV4=y
5 # native_sim uses the host's network stack via a TAP interface or SLIP
6 CONFIG_NET_NATIVE_OFFLOADED_SOCKETS_ANY=y 2. UDP Bridge Thread (src/bridge/gazebo_bridge.cpp):

- Implement a thread that opens a UDP socket on port 5005 (TX) and 5006 (RX).
  \_ TX: Periodically sends a JSON string: {"left_pwm": 128, "right_pwm": -64}. \* RX: Listens for sensor updates: {"l_ticks": 1024, "r_ticks": 2048, "imu": [x,y,z]}.

B. Hardware Abstraction: Motor and Encoders
Refactor src/actuator/motor.cpp and src/sensors/encoders.cpp to use conditional compilation:

    1 #if defined(CONFIG_BOARD_NATIVE_SIM)
    2     // Virtual Implementation
    3     void Motor::set_motor(Direction dir, uint8_t pwm_val) {
    4         GazeboBridge::send_motor_cmd(name_, dir, pwm_val);
    5     }
    6     int32_t Encoders::get_total_ticks() {
    7         return GazeboBridge::get_virtual_ticks(unit_idx_);
    8     }
    9 #else

10 // Existing Hardware Implementation (PWM/PCNT)
11 void Motor::set*motor(Direction dir, uint8_t pwm_val) {
12 pwm_set_pulse_dt(fwd_pwm*, pulse); ...
13 }
14 #endif

C. Gazebo Side: Python Bridge (simulation/bridge)
Provide a script using gz-transport (Gazebo Sim) or rospy (if using ROS):

1.  Listen (UDP 5005): Convert PWM values to cmd_vel or ApplyJointForce.
2.  Publish (UDP 5006): Read joint states (encoders) and IMU data from Gazebo and stream them back to Zephyr.

D. Build System
Update CMakeLists.txt to include the bridge only for native_sim:
1 if(CONFIG_BOARD_NATIVE_SIM)
2 target_sources(app PRIVATE src/bridge/gazebo_bridge.cpp)
3 endif()

---

3. Key Commands for your Environment

To build and run the simulation:

1.  Build for Simulation:
    1 west build -p -b native_sim zephyr-port/robot_app
2.  Run Zephyr:
    1 ./build/zephyr/zephyr.exe
3.  Launch Gazebo Bridge:
    1 export PYTHONPATH=$PYTHONPATH:$(pwd)/simulation/bridge/src
    2 python3 -m gazebo_bridge.main

4.  Pro-Tip: JSON vs. Protobuf for the Bridge
    Since your project already uses Protobuf (messages.proto), I recommend using the same Protobuf messages over UDP
    for the Gazebo bridge. This ensures type safety and allows you to reuse your ProtobufHandler logic in the
    simulation, making the "Virtual" drivers even thinner.
