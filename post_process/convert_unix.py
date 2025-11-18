#!/usr/bin/env python3

import rosbag
import time
import os
import datetime

def unix_to_yyyymmddhhmmssmmm(unix_ts):
    """
    Converts a Unix timestamp (float) to a string in format: YYYYMMDDHHMMSSmmm
    """
    dt = datetime.datetime.utcfromtimestamp(unix_ts)
    milliseconds = int((unix_ts - int(unix_ts)) * 1000)
    return dt.strftime('%Y%m%d%H%M%S') + f"{milliseconds:03d}"

def extract_gps_with_converted_timestamps(bag_file, gps_topic):
    """
    Extracts GPS data from ROS bag, converts timestamps to YYYYMMDDHHMMSSmmm,
    and saves to a file named with the current Unix timestamp.
    
    Output format:
        #timestamp latitude longitude altitude
    """
    # Output filename based on script execution time (Unix seconds)
    unix_now = int(time.time())
    output_dir = os.path.dirname(bag_file)
    output_file = os.path.join(output_dir, f"gps_data.txt")

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

    print(f"GPS data with converted timestamps saved to: {output_file}")
    return output_file


if __name__ == "__main__":
    BAG_FILE = "/media/kodifly/Extreme Pro/2025-11-11-rosbags/_2025-11-11-13-55-48.bag"
    GPS_TOPIC = "/gps/fix"

    extract_gps_with_converted_timestamps(BAG_FILE, GPS_TOPIC)