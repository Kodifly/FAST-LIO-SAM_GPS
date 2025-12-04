#!/usr/bin/env python3

import rosbag
import os
import datetime
import argparse
import shutil
import tempfile

# ---------- GPS Extraction ----------
def unix_to_yyyymmddhhmmssmmm(unix_ts):
    """Converts a Unix timestamp (float) to YYYYMMDDHHMMSSmmm string."""
    dt = datetime.datetime.utcfromtimestamp(unix_ts)
    milliseconds = int((unix_ts - int(unix_ts)) * 1000)
    return dt.strftime('%Y%m%d%H%M%S') + f"{milliseconds:03d}"

def extract_gps_lines(bag_file, gps_topic):
    """Extract GPS data as list of lines (without writing to file)."""
    lines = ["#timestamp latitude longitude altitude\n"]
    bag = rosbag.Bag(bag_file, "r")
    for topic, msg, t in bag.read_messages(topics=[gps_topic]):
        try:
            unix_ts = msg.header.stamp.to_sec()
            formatted_ts = unix_to_yyyymmddhhmmssmmm(unix_ts)
            lat = msg.latitude
            lon = msg.longitude
            alt = round(msg.altitude, 3)
            lines.append(f"{formatted_ts} {lat} {lon} {alt}\n")
        except Exception as e:
            print(f"Error processing GPS message: {e}")
    bag.close()
    return lines

# ---------- Timestamp Alignment ----------
def get_first_timestamp_from_lines(lines):
    """Reads the first valid numeric timestamp from a list of lines."""
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith('#'):
            continue
        try:
            return int(stripped.split()[0])
        except (ValueError, IndexError):
            continue
    raise ValueError("No valid timestamp found in provided lines")

def get_first_timestamp(filepath):
    """Reads the first valid numeric timestamp from a file."""
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            try:
                return int(line.split()[0])
            except (ValueError, IndexError):
                continue
    raise ValueError(f"No valid timestamp found in {filepath}")

def align_gps_lines_to_slam(gps_lines, slam_file, output_file):
    """Align GPS lines to SLAM file’s first timestamp and write to output_file."""
    try:
        gps_start = get_first_timestamp_from_lines(gps_lines)
        slam_start = get_first_timestamp(slam_file)
    except ValueError as e:
        print(f"Error: {e}")
        return False

    offset = slam_start - gps_start
    print(f"GPS start:  {gps_start}")
    print(f"SLAM start: {slam_start}")
    print(f"Offset (SLAM - GPS): {offset}")

    with open(output_file, 'w') as outfile:
        for line in gps_lines:
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                outfile.write(line)
                continue

            parts = stripped.split()
            try:
                orig_ts = int(parts[0])
                new_ts = orig_ts + offset
                parts[0] = str(new_ts)
                outfile.write(' '.join(parts) + '\n')
            except (ValueError, IndexError):
                outfile.write(line)

    print(f"Aligned GPS saved to: {output_file}")
    return True

# ---------- Main ----------
def main():
    parser = argparse.ArgumentParser(
        description="Extract GPS from ROS bag and align it to a SLAM poses file. Only aligned GPS is saved as gps_data.txt."
    )
    parser.add_argument(
        "--bag", required=True,
        help="Path to input ROS bag file"
    )
    parser.add_argument(
        "--gps-topic", default="/gps/fix",
        help="GPS topic name (default: /gps/fix)"
    )
    parser.add_argument(
        "--slam-file", default=None,
        help="Path to SLAM poses file. If provided, GPS will be aligned to it."
    )
    parser.add_argument(
        "--output-dir", default=None,
        help="Directory to save gps_data.txt (and copy of SLAM file). Default: bag file's directory"
    )

    args = parser.parse_args()

    output_dir = args.output_dir if args.output_dir else os.path.dirname(args.bag)
    os.makedirs(output_dir, exist_ok=True)

    # Step 1: Extract GPS into memory
    gps_lines = extract_gps_lines(args.bag, args.gps_topic)

    gps_output_path = os.path.join(output_dir, "gps_data.txt")

    if args.slam_file:
        if not os.path.isfile(args.slam_file):
            print(f"Error: SLAM file not found: {args.slam_file}")
            return

        # Align GPS to SLAM and write directly to gps_data.txt
        success = align_gps_lines_to_slam(gps_lines, args.slam_file, gps_output_path)
        if not success:
            return

        # Copy SLAM file as poses_tum_converted.txt
        try:
            dst_slam_name = os.path.join(output_dir, "poses_tum_converted.txt")
            shutil.copyfile(args.slam_file, dst_slam_name)
            print(f"Copied SLAM file to: {dst_slam_name}")

            # Remove the original SLAM file after successful copy
            os.remove(args.slam_file)
            print(f"Removed original SLAM file: {args.slam_file}")
        except Exception as e:
            print(f"Warning: failed to copy or remove SLAM file: {e}")

    else:
        # No alignment: just write raw GPS lines to gps_data.txt
        with open(gps_output_path, 'w') as f:
            f.writelines(gps_lines)
        print(f"GPS data saved to: {gps_output_path}")

if __name__ == "__main__":
    main()