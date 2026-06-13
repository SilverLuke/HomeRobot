# Plan: Evaluation and Migration of Robot Proxy Strategy

## 1. Objective
To evaluate the current "Smart Server, Dumb Robot" architecture and propose a robust, industry-standard solution for the "Robot Proxy" (Bouncer) component. The goal is to ensure the robot remains connected while the control logic is updated or restarted, minimizing downtime and latency.

## 2. The "Bouncer" Pattern: Rationale
Decoupling the physical connection from the application logic is a best practice in robotics for:
- **Zero-Downtime Logic Updates**: Restarting the Rust `server` without triggering a robot Wi-Fi reconnection (saving 10-30s per iteration).
- **Tool Interoperability**: Allowing `cmd_sender`, `server`, or diagnostic tools to "swap" control of the robot instantly.
- **Connection Resilience**: Buffering small amounts of data or simply holding the socket open during transient backend failures.

## 3. Comparison of Industry-Standard Tools

| Tool | Type | Suitability for HomeRobot | Key Characteristics |
| :--- | :--- | :--- | :--- |
| **Nginx (Stream)** | Reverse Proxy | **High** | Lightweight, high performance for raw TCP, extremely stable. |
| **HAProxy** | Load Balancer | **High** | Industry standard for TCP proxying; handles complex "holding" queues. |
| **socat** | CLI Utility | **Medium** | Excellent for debugging, but requires shell wrappers for persistence. |
| **Apache** | Web Server | **Low** | Overkill for raw TCP; designed for HTTP request-response cycles. |
| **MQTT** | Broker | **Strategic** | Best for scale, but requires refactoring firmware to use MQTT instead of raw TCP. |

## 4. Proposed Strategy: Nginx Stream Module
Replacing the custom Rust `robot_proxy` with Nginx is the recommended path for a stable, "already developed" solution.

### Benefits
- **Performance**: Eliminates the current 5ms polling lag in the custom Rust proxy.
- **Reliability**: Zero-maintenance; Nginx is battle-tested.
- **Local Integration**: Can be easily launched via a simple configuration file and a Makefile target.

## 5. Implementation Roadmap (Draft)

### Step 1: Configuration
Create `tools/nginx/robot_proxy.conf` with a `stream` block:
- **Listen**: 12345 (Robot Port)
- **Proxy Pass**: 127.0.0.1:12346 (Controller Port)
- **Timeouts**: Long `proxy_timeout` to prevent idle robot disconnects.

### Step 2: Makefile Integration
Add a `make proxy` command to launch the Nginx proxy using the local config.
```bash
nginx -c $(PWD)/tools/nginx/robot_proxy.conf -g "daemon off;"
```

### Step 3: Verification
- Verify robot connectivity to port 12345.
- Verify that killing/starting the `server` on port 12346 does not disconnect the robot.
- Measure latency to ensure it is near-zero.

---
**Note**: This document is for planning and architectural review only. No implementation actions have been taken.
