# Idea: G-code style RPC Motion Control

## Problem Statement
How might we implement precise, blocking kinematic commands (straight lines, rotations, and arcs) that leverage the ESP32's local PID ticks controller to eliminate network latency errors while providing a clean, scripting-friendly API on the server?

## Recommended Direction
We will implement an RPC-based motion command interface. The server will calculate target wheel ticks based on physical constants (wheelbase, ticks-per-meter) and issue an `ExecuteMotion` RPC request. 

The ESP32 firmware will receive this request, set the motor PID targets locally, and wait until both motors have reached their targets (or a safety timeout is triggered) before sending the RPC response. This creates a natural, blocking G-code flow on the server side:

```rust
// Example server-side scripting:
robot.execute_motion(MotionType::Straight, 1.0)?; // Moves 1 meter, blocks until done
robot.execute_motion(MotionType::Rotate, 90.0)?;  // Rotates 90 degrees, blocks until done
```

---

## Technical Design

### 1. Protobuf Changes
We will define a new `MotionRequest` message in `messages.proto` to act as the payload for the `ExecuteMotion` RPC:

```protobuf
message MotionRequest {
  enum Type {
    STRAIGHT = 0;
    ROTATE = 1;
    ARC = 2;
  }
  Type type = 1;
  int32 left_ticks = 2;
  int32 right_ticks = 3;
  uint32 max_power = 4; // Power limit (0-255) for the move
}
```

### 2. ESP32 Execution Flow (C++)
1. Receive `ExecuteMotion` RPC request.
2. Store the `call_id` and the motion request.
3. Reset current encoder positions to 0.
4. Set motor PID targets using `motor_sx_.set_target(left_ticks)` and `motor_dx_.set_target(right_ticks)`.
5. In the main loop, check if both motors have reached their targets using `target_reached(true)`.
6. Once reached, send the `RpcResponse` back to the server containing the stored `call_id`.

---

## Key Assumptions to Validate
- [ ] **Zephyr PCNT Reset stability:** Verify that resetting encoder ticks to 0 mid-run does not cause hiccups in the hardware PCNT reader.
- [ ] **PID tuning consistency:** Verify that a single set of Kp, Ki, Kd parameters can handle both high-speed straight lines and fine-angle rotations.
- [ ] **Timeout handling:** Ensure the robot returns an RPC error response if it gets physically stuck and cannot reach the tick targets within a reasonable timeout.

## MVP Scope
* **In-scope:**
  * Implementing the `MotionRequest` protobuf definition.
  * Modifying the ESP32 firmware to process `ExecuteMotion` RPC calls and run the local PID controller.
  * Adding Rust server API wrappers to trigger these motion commands and await their completion.
  * Basic straight line and rotation support.
* **Out-of-scope for MVP:**
  * Complex acceleration/deceleration profiling (trapezoidal profiling).
  * Coordinated arc/curve paths (added in phase 2 after primitives are verified).

## Not Doing (and Why)
* **Real-time path adjustment during RPC:** Since the RPC blocks until completion, we cannot easily adjust the targets mid-flight. If an obstacle is detected, the server must issue a separate `StopAll` command to abort the active move.

## Open Questions
* Should the server enforce a safety timeout, or should the ESP32 handle the timeout internally to guarantee that the RPC connection never hangs indefinitely?
