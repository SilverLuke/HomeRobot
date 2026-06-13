#!/usr/bin/env python3
import sys
import json
import subprocess
import os
import time

def log_to_stderr(msg):
    sys.stderr.write(f"DEBUG: {msg}\n")
    sys.stderr.flush()

def run_command(cmd, shell=True):
    log_to_stderr(f"Running command: {cmd}")
    result = subprocess.run(cmd, shell=shell, capture_output=True, text=True)
    return {
        "stdout": result.stdout,
        "stderr": result.stderr,
        "exit_code": result.returncode
    }

def get_logs(duration=2):
    log_to_stderr(f"Fetching logs for {duration}s")
    # Assuming tools/read_esp32_logs.sh exists and works as described in GEMINI.md
    res = run_command(f"./tools/read_esp32_logs.sh {duration}")
    return res

def flash_robot():
    log_to_stderr("Flashing robot...")
    return run_command("make flash")

def build_robot():
    log_to_stderr("Building robot...")
    return run_command("make build-c6")

def send_robot_move(left, right, l_angle=1.0, r_angle=1.0):
    log_to_stderr(f"Sending move: L={left}, R={right}")
    # Using cmd_sender tool. Note: it might need to connect to a running proxy or robot.
    # cmd_sender move --left 100 --right 100
    cmd = f"cargo run --manifest-path tools/cmd_sender/Cargo.toml -- move --left {left} --right {right} --l-angle {l_angle} --r-angle {r_angle}"
    return run_command(cmd)

def run_diagnostics():
    log_to_stderr("Running diagnostics...")
    cmd = "cargo run --manifest-path tools/cmd_sender/Cargo.toml -- diag"
    return run_command(cmd)

def handle_request(request):
    method = request.get("method")
    params = request.get("params", {})
    id = request.get("id")

    log_to_stderr(f"Handling request: {method}")

    result = None
    if method == "initialize":
        result = {
            "capabilities": {
                "tools": [
                    {"name": "build_robot", "description": "Build the robot firmware"},
                    {"name": "flash_robot", "description": "Flash the robot firmware"},
                    {"name": "get_logs", "description": "Get ESP32 logs", "parameters": {"type": "object", "properties": {"duration": {"type": "integer"}}}},
                    {"name": "move_robot", "description": "Move the robot", "parameters": {"type": "object", "properties": {"left": {"type": "integer"}, "right": {"type": "integer"}}}},
                    {"name": "run_diagnostics", "description": "Run hardware diagnostics"}
                ]
            }
        }
    elif method == "tools/call":
        tool_name = params.get("name")
        tool_args = params.get("arguments", {})
        
        if tool_name == "build_robot":
            result = {"content": [{"type": "text", "text": json.dumps(build_robot())}]}
        elif tool_name == "flash_robot":
            result = {"content": [{"type": "text", "text": json.dumps(flash_robot())}]}
        elif tool_name == "get_logs":
            result = {"content": [{"type": "text", "text": json.dumps(get_logs(tool_args.get("duration", 2)))}]}
        elif tool_name == "move_robot":
            result = {"content": [{"type": "text", "text": json.dumps(send_robot_move(tool_args.get("left", 0), tool_args.get("right", 0)))}]}
        elif tool_name == "run_diagnostics":
            result = {"content": [{"type": "text", "text": json.dumps(run_diagnostics())}]}
        else:
            result = {"error": {"code": -32601, "message": "Method not found"}}
    
    return {"jsonrpc": "2.0", "id": id, "result": result}

def main():
    log_to_stderr("Robot MCP Server started")
    for line in sys.stdin:
        try:
            request = json.loads(line)
            response = handle_request(request)
            sys.stdout.write(json.dumps(response) + "\n")
            sys.stdout.flush()
        except Exception as e:
            log_to_stderr(f"Error: {e}")

if __name__ == "__main__":
    main()
