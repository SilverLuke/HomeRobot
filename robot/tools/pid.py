import re
import matplotlib.pyplot as plt
import time
import os
from matplotlib.animation import FuncAnimation

# Set your absolute file path here
file_path = "/home/luca/Projects/robotica/homeRobot/software/robot/serial.log"

def parse_file():
    """Parse the file and return the data"""
    # Initialize storage
    dx_data = {'Time': [], 'Position': [], 'Target': [], 'Error': [], 'ekp': [], 'eki': [], 'ekd': [], 'Power': []}
    sx_data = {'Time': [], 'Position': [], 'Target': [], 'Error': [], 'ekp': [], 'eki': [], 'ekd': [], 'Power': []}
    reached_times = []  # Store times when Reached changes from 0 to 1
    dx_prev_reached = None  # Track previous reached state for DX motor
    sx_prev_reached = None  # Track previous reached state for SX motor
    parsed_lines = 0

    try:
        # Read the file
        with open(file_path, 'r') as f:
            lines = f.readlines()
    except FileNotFoundError:
        print(f"File not found: {file_path}")
        return dx_data, sx_data, reached_times, parsed_lines

    # Regular expression to extract fields (updated to include Direction)
    pattern = r'(\d+):(\d+):(\d+):(\d+\.\d+).*?(DX|SX): Position: (-?\d+) Target: (-?\d+) Reached: (0|1) Error: (-?\d+) Ekp: (-?\d+\.\d+) Eki: (-?\d+\.\d+) Ekd: (-?\d+\.\d+) Direction: (FORWARD|BACKWARD|BRAKE|FREE) Power: (\d+)'
    for line in lines:
        parsed_lines += 1
        
        # Check if we found the stop command
        if "Serial cmd: Stop" in line:
            print(f"Found 'Serial cmd: Stop' at line {parsed_lines}. Stopping parsing.")
            break
        
        match = re.match(pattern, line)
        if match:
            day, hour, minute, second, motor, position, target, reached, error, ekp, eki, ekd, direction, power = match.groups()
            # Convert time to total seconds
            total_time = (
                int(day) * 86400 +
                int(hour) * 3600 +
                int(minute) * 60 +
                float(second)
            )
            position = int(position)
            target = int(target)
            reached_val = int(reached)
            error = int(error)
            power = int(power)
            ekp = float(ekp)
            eki = float(eki)
            ekd = float(ekd)
            
            # Check for reached state change (0 -> 1) for each motor separately
            if motor == "DX":
                if dx_prev_reached is not None and dx_prev_reached == 0 and reached_val == 1:
                    reached_times.append(total_time)
                dx_prev_reached = reached_val
            elif motor == "SX":
                if sx_prev_reached is not None and sx_prev_reached == 0 and reached_val == 1:
                    reached_times.append(total_time)
                sx_prev_reached = reached_val
            
            # Apply direction to power
            if direction == "FORWARD":
                power = power * 1  # Multiply by 1 for forward
            elif direction == "BACKWARD":
                power = power * -1  # Multiply by -1 for backward
            else:
                power = 0

            if motor == "DX":
                dx_data['Time'].append(total_time)
                dx_data['Position'].append(position)
                dx_data['Target'].append(target)
                dx_data['Error'].append(error)
                dx_data['ekp'].append(ekp)
                dx_data['eki'].append(eki)
                dx_data['ekd'].append(ekd)
                dx_data['Power'].append(power)
            elif motor == "SX":
                sx_data['Time'].append(total_time)
                sx_data['Position'].append(position)
                sx_data['Target'].append(target)
                sx_data['Error'].append(error)
                sx_data['ekp'].append(ekp)
                sx_data['eki'].append(eki)
                sx_data['ekd'].append(ekd)
                sx_data['Power'].append(power)

    return dx_data, sx_data, reached_times, parsed_lines

# Global variables for file monitoring
last_modified_time = 0

def add_reference_lines(ax, times):
    """Add vertical lines at specified times and horizontal line at y=0"""
    # Add vertical lines for reached times
    for time in times:
        ax.axvline(x=time, color='green', linestyle='--', alpha=0.7, linewidth=1)
    # Add horizontal line at y=0
    ax.axhline(y=0, color='black', linestyle='-', alpha=0.5, linewidth=0.8)

def plot_param(ax, dx_time, dx_vals, sx_time, sx_vals, ylabel, reached_times, color_dx='blue', color_sx='red'):
    ax.clear()
    ax.plot(dx_time, dx_vals, label=f'DX {ylabel}', color=color_dx)
    ax.plot(sx_time, sx_vals, label=f'SX {ylabel}', color=color_sx)
    ax.set_ylabel(ylabel)
    ax.legend()
    add_reference_lines(ax, reached_times)

def plot_position_and_target(ax, dx_data, sx_data, reached_times):
    ax.clear()
    # Plot Position
    ax.plot(dx_data['Time'], dx_data['Position'], label='DX Position', color='blue', linewidth=2)
    ax.plot(sx_data['Time'], sx_data['Position'], label='SX Position', color='red', linewidth=2)
    
    # Plot Target
    ax.plot(dx_data['Time'], dx_data['Target'], '--', label='DX Target', color='lightblue', linewidth=1.5)
    ax.plot(sx_data['Time'], sx_data['Target'], '--', label='SX Target', color='lightcoral', linewidth=1.5)
    
    ax.set_ylabel('Position/Target')
    ax.legend()
    ax.set_title('Position and Target Over Time')
    add_reference_lines(ax, reached_times)

def plot_error(ax, dx_data, sx_data, reached_times):
    ax.clear()
    ax.plot(dx_data['Time'], dx_data['Error'], label='DX Error', color='cyan', linewidth=2)
    ax.plot(sx_data['Time'], sx_data['Error'], label='SX Error', color='orange', linewidth=2)
    ax.set_ylabel('Error')
    ax.legend()
    ax.set_title('Error Over Time')
    add_reference_lines(ax, reached_times)

def update_plots(frame):
    """Update function for animation - checks for file changes and redraws plots"""
    global last_modified_time
    
    try:
        current_modified_time = os.path.getmtime(file_path)
        
        # Check if file has been modified
        if current_modified_time > last_modified_time:
            last_modified_time = current_modified_time
            print(f"File changed, reloading data... (frame {frame})")
            
            # Parse the updated file
            dx_data, sx_data, reached_times, parsed_lines = parse_file()
            
            print(f"Total lines parsed: {parsed_lines}")
            print(f"Found {len(reached_times)} times when Reached changed from 0 to 1")
            
            # Clear and redraw all plots
            plot_position_and_target(axs[0], dx_data, sx_data, reached_times)
            plot_error(axs[1], dx_data, sx_data, reached_times)
            plot_param(axs[2], dx_data['Time'], dx_data['Power'], sx_data['Time'], sx_data['Power'], 'Power (Directional)', reached_times)
            plot_param(axs[3], dx_data['Time'], dx_data['ekp'], sx_data['Time'], sx_data['ekp'], 'Ekp', reached_times)
            plot_param(axs[4], dx_data['Time'], dx_data['eki'], sx_data['Time'], sx_data['eki'], 'Eki', reached_times)
            plot_param(axs[5], dx_data['Time'], dx_data['ekd'], sx_data['Time'], sx_data['ekd'], 'Ekd', reached_times)
            
            axs[-1].set_xlabel('Time Step')
            
    except FileNotFoundError:
        print(f"File not found: {file_path}")
    except Exception as e:
        print(f"Error updating plots: {e}")

# Initial setup
print("Starting automatic file monitoring...")

# Get initial file modification time
try:
    last_modified_time = os.path.getmtime(file_path)
except FileNotFoundError:
    print(f"File not found: {file_path}")
    last_modified_time = 0

# Create subplots
fig, axs = plt.subplots(6, 1, figsize=(14, 20), sharex=True)
fig.suptitle('Motor Data Over Time (Auto-Reloading)')

# Initial plot
dx_data, sx_data, reached_times, parsed_lines = parse_file()
print(f"Initial load - Total lines parsed: {parsed_lines}")
print(f"Found {len(reached_times)} times when Reached changed from 0 to 1")

# Plot initial data
plot_position_and_target(axs[0], dx_data, sx_data, reached_times)
plot_error(axs[1], dx_data, sx_data, reached_times)
plot_param(axs[2], dx_data['Time'], dx_data['Power'], sx_data['Time'], sx_data['Power'], 'Power (Directional)', reached_times)
plot_param(axs[3], dx_data['Time'], dx_data['ekp'], sx_data['Time'], sx_data['ekp'], 'Ekp', reached_times)
plot_param(axs[4], dx_data['Time'], dx_data['eki'], sx_data['Time'], sx_data['eki'], 'Eki', reached_times)
plot_param(axs[5], dx_data['Time'], dx_data['ekd'], sx_data['Time'], sx_data['ekd'], 'Ekd', reached_times)

axs[-1].set_xlabel('Time Step')
plt.tight_layout()

# Set up animation to check for file changes every 1000ms (1 second)
ani = FuncAnimation(fig, update_plots, interval=1000, cache_frame_data=False)

print("Monitoring file for changes. Close the plot window to stop.")
plt.show()