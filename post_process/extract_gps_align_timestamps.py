#!/usr/bin/env python3

import rosbag
import os
import datetime
import argparse
import shutil                    # <-- added

# ---------- GPS Extraction ----------
def unix_to_yyyymmddhhmmssmmm(unix_ts):
    """Converts a Unix timestamp (float) to YYYYMMDDHHMMSSmmm string."""
    dt = datetime.datetime.utcfromtimestamp(unix_ts)
    milliseconds = int((unix_ts - int(unix_ts)) * 1000)
    return dt.strftime('%Y%m%d%H%M%S') + f"{milliseconds:03d}"

def extract_gps_with_converted_timestamps(bag_file, gps_topic, output_dir=None):
    """Extract GPS data, convert timestamps, save as gps_data.txt."""
    if output_dir is None:
        output_dir = os.path.dirname(bag_file)
    output_file = os.path.join(output_dir, "gps_data.txt")

    with open(output_file, 'w') as f:
        f.write("#timestamp latitude longitude altitude\n")
        bag = rosbag.Bag(bag_file, "r")
        for topic, msg, t in bag.read_messages(topics=[gps_topic]):
            try:
                unix_ts = msg.header.stamp.to_sec()
                formatted_ts = unix_to_yyyymmddhhmmssmmm(unix_ts)
                lat = msg.latitude
                lon = msg.longitude
                alt = round(msg.altitude, 3)
                f.write(f"{formatted_ts} {lat} {lon} {alt}\n")
            except Exception as e:
                print(f"Error processing GPS message: {e}")
        bag.close()

    print(f"GPS data saved to: {output_file}")
    return output_file

# ---------- Timestamp Alignment ----------
def get_first_timestamp(filepath):
    """Reads the first valid numeric timestamp from a file (ignores comments/empty lines)."""
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

def align_slam_to_gps(slam_file, gps_file, output_file):
    """Shift SLAM timestamps so that its first timestamp matches GPS first timestamp."""
    try:
        slam_start = get_first_timestamp(slam_file)
        gps_start = get_first_timestamp(gps_file)
    except ValueError as e:
        print(f"Error: {e}")
        return False

    offset = gps_start - slam_start
    print(f"SLAM start: {slam_start}")
    print(f"GPS start:  {gps_start}")
    print(f"Offset (GPS - SLAM): {offset}")

    with open(slam_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line in infile:
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
                outfile.write(line)  # keep malformed lines

    print(f"Aligned SLAM file saved to: {output_file}")
    return True

def align_gps_to_slam(gps_file, slam_file, output_file):
    """Shift GPS timestamps so that its first timestamp matches SLAM first timestamp."""
    try:
        gps_start = get_first_timestamp(gps_file)
        slam_start = get_first_timestamp(slam_file)
    except ValueError as e:
        print(f"Error: {e}")
        return False

    offset = slam_start - gps_start
    print(f"GPS start:  {gps_start}")
    print(f"SLAM start: {slam_start}")
    print(f"Offset (SLAM - GPS): {offset}")

    with open(gps_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line in infile:
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

    print(f"Aligned GPS file saved to: {output_file}")
    return True

# ---------- Main ----------
def main():
    parser = argparse.ArgumentParser(
        description="Extract GPS from ROS bag and optionally align a SLAM poses file to it."
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
        help="Path to SLAM poses file (e.g., slam_poses_20251111135548600_20251111135559900.txt). If provided, alignment will be performed."
    )
    parser.add_argument(
        "--output-dir", default=None,
        help="Directory to save gps_data.txt and aligned SLAM file (default: bag file's directory)"
    )

    args = parser.parse_args()

    # Determine output directory
    output_dir = args.output_dir if args.output_dir else os.path.dirname(args.bag)

    # Step 1: Extract GPS
    gps_file = extract_gps_with_converted_timestamps(
        bag_file=args.bag,
        gps_topic=args.gps_topic,
        output_dir=output_dir
    )

    # Step 2: Align SLAM if provided
    if args.slam_file:
        if not os.path.isfile(args.slam_file):
            print(f"Error: SLAM file not found: {args.slam_file}")
            return

        # slam_out = os.path.join(output_dir, "poses_tum_converted.txt")
        # align_slam_to_gps(args.slam_file, gps_file, slam_out)
        gps_out = os.path.join(output_dir, "gps_data_aligned.txt")
        ok = align_gps_to_slam(gps_file, args.slam_file, gps_out)

        # after processing, copy/rename provided SLAM file to "poses_tum_converted.txt" in output_dir
        try:
            dst_slam_name = os.path.join(output_dir, "poses_tum_converted.txt")
            shutil.copyfile(args.slam_file, dst_slam_name)
            print(f"Copied SLAM file to: {dst_slam_name}")
        except Exception as e:
            print(f"Warning: failed to copy SLAM file to poses_tum_converted.txt: {e}")
    else:
        print("No SLAM file provided — skipping alignment.")

if __name__ == "__main__":
    main()