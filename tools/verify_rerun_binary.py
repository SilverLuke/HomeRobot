import sys

def verify_rrd(file_path):
    print(f"Verifying {file_path}...")
    try:
        with open(file_path, 'rb') as f:
            content = f.read()
            
        streams = [
            b"robot/sensor/lidar",
            b"robot/pose/slam",
            b"robot/exploration/frontiers",
            b"robot/exploration/path"
        ]
        
        all_found = True
        for stream in streams:
            if stream in content:
                print(f"  [✓] Found stream: {stream.decode()}")
            else:
                print(f"  [✗] MISSING stream: {stream.decode()}")
                all_found = False
        
        return all_found
    except Exception as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    if verify_rrd("logs/sim/simulation.rrd"):
        print("\nCONCLUSION: Rerun integration is WORKING perfectly.")
        sys.exit(0)
    else:
        print("\nCONCLUSION: Rerun integration has MISSING streams.")
        sys.exit(1)
