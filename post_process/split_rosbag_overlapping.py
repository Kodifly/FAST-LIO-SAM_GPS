#!/usr/bin/env python3

import rosbag
import rospy
import os
import sys
from datetime import timedelta

def split_rosbag_with_overlap(input_bag_path, output_dir, segment_duration_sec=300, overlap_sec=60):
    """
    Split a rosbag into segments of `segment_duration_sec` seconds,
    with `overlap_sec` seconds overlapping between consecutive segments.
    
    Example: 5 min = 300 sec, overlap = 60 sec
    Segments: [0, 300), [240, 540), [480, 780), ...
    """
    if not os.path.isfile(input_bag_path):
        raise FileNotFoundError(f"Input bag file not found: {input_bag_path}")
    
    os.makedirs(output_dir, exist_ok=True)

    with rosbag.Bag(input_bag_path, 'r') as in_bag:
        start_time = in_bag.get_start_time()  # in seconds (float)
        end_time = in_bag.get_end_time()

        if end_time <= start_time:
            print("Bag file is empty or invalid.")
            return

        segment_step = segment_duration_sec - overlap_sec  # e.g., 300 - 60 = 240 sec
        current_start = start_time
        segment_index = 0

        while current_start < end_time:
            current_end = current_start + segment_duration_sec
            if current_start >= end_time:
                break

            # Clip end to actual bag end
            actual_end = min(current_end, end_time)

            # Only create segment if it contains data
            if actual_end > current_start:
                out_filename = os.path.join(
                    output_dir,
                    f"segment_{segment_index:04d}_{int(current_start - start_time)}s_to_{int(actual_end - start_time)}s.bag"
                )
                print(f"Writing segment {segment_index}: {current_start - start_time:.1f}s to {actual_end - start_time:.1f}s -> {out_filename}")

                with rosbag.Bag(out_filename, 'w') as out_bag:
                    for topic, msg, t in in_bag.read_messages(
                        start_time=rospy.Time.from_sec(current_start),
                        end_time=rospy.Time.from_sec(actual_end)
                    ):
                        out_bag.write(topic, msg, t)

            segment_index += 1
            current_start += segment_step

    print("Splitting completed.")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python3 split_rosbag_overlapping.py <input.bag> <output_directory>")
        sys.exit(1)

    input_bag = sys.argv[1]
    output_dir = sys.argv[2]

    split_rosbag_with_overlap(
        input_bag_path=input_bag,
        output_dir=output_dir,
        segment_duration_sec=300,   # 5 minutes
        overlap_sec=60              # 1 minute overlap
    )