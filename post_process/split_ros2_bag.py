import argparse
from pathlib import Path
from rosbags.rosbag2 import Reader, Writer

def extract_by_duration(input_bag: Path, output_bag: Path, duration_sec: float):
    """
    Extracts the initial duration of a ROS 2 bag file into a new bag.

    Args:
        input_bag: Path to the input rosbag2 directory.
        output_bag: Path to the output rosbag2 directory to be created.
        duration_sec: The duration in seconds to extract from the beginning of the bag.
    """
    if not input_bag.is_dir():
        print(f"Error: Input bag path '{input_bag}' is not a directory.")
        return

    # Create parent directory for the output bag if it doesn't exist
    output_bag.parent.mkdir(parents=True, exist_ok=True)

    with Reader(input_bag) as reader:
        # The start time of the bag is the timestamp of the first message
        start_time_ns = reader.start_time
        duration_ns = int(duration_sec * 1_000_000_000)
        end_time_ns = start_time_ns + duration_ns
        
        print(f"Bag start time (ns): {start_time_ns}")
        print(f"Extracting for {duration_sec} seconds until timestamp (ns): {end_time_ns}")

        with Writer(output_bag) as writer:
            # Copy the connection (topic) information
            connections = {}
            for connection in reader.connections:
                # --- FIX STARTS HERE ---
                # Manually specify the serialization format, as it's standard for .db3 files.
                # Use getattr for QoS profiles for better compatibility with different library versions.
                connections[connection.id] = writer.add_connection(
                    topic=connection.topic,
                    msgtype=connection.msgtype,
                    serialization_format='cdr',
                    offered_qos_profiles=getattr(connection, 'offered_qos_profiles', '')
                )
                # --- FIX ENDS HERE ---

            # Iterate through messages and write them if they are within the time window
            for connection, timestamp, rawdata in reader.messages():
                if timestamp <= end_time_ns:
                    # Use the new connection object from the writer's dictionary
                    writer.write(connections[connection.id], timestamp, rawdata)
                else:
                    # Messages are typically ordered by time, so we can stop
                    # once we are past the desired end time.
                    print("Reached end of duration. Stopping extraction.")
                    break
    
    print(f"\nExtraction complete. New bag saved to: {output_bag}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract an initial duration from a ROS 2 bag file.")
    parser.add_argument("input_bag", type=Path, help="Path to the input rosbag2 directory.")
    parser.add_argument("output_bag", type=Path, help="Path for the new output rosbag2 directory.")
    parser.add_argument("--duration", type=float, default=100.0, help="Duration in seconds to extract from the start of the bag.")

    args = parser.parse_args()
    # Ensure the script name in the traceback matches your saved file name
    # The original was split_ros2_bag.py, let's assume it's now extract_by_duration.py
    extract_by_duration(args.input_bag, args.output_bag, args.duration)