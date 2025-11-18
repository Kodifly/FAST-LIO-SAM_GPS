#!/usr/bin/env python3

import rosbag
import time
import os

def extract_gps_to_unix_named_file(bag_file, gps_topic):
    """
    Extracts GPS data from a ROS bag and saves it to a file named with the current Unix timestamp.
    
    Output format:
        #timestamp latitude longitude altitude
    """
    # Generate output filename using current Unix timestamp (integer seconds)
    unix_ts = int(time.time())
    output_dir = os.path.dirname(bag_file)
    output_file = os.path.join(output_dir, f"gps_{unix_ts}.txt")

    with open(output_file, 'w') as f:
        f.write("#timestamp latitude longitude altitude\n")

        bag = rosbag.Bag(bag_file, "r")
        for topic, msg, t in bag.read_messages(topics=[gps_topic]):
            try:
                timestamp = msg.header.stamp.to_sec()  # Already Unix time (float)
                lat = msg.latitude
                lon = msg.longitude
                alt = round(msg.altitude, 3)

                f.write(f"{timestamp} {lat} {lon} {alt}\n")
            except Exception as e:
                print(f"Error processing GPS message: {e}")
        bag.close()

    print(f"GPS data saved to: {output_file}")
    return output_file


if __name__ == "__main__":
    BAG_FILE = "/media/kodifly/Extreme Pro/2025-11-11-rosbags/_2025-11-11-13-55-48.bag"
    GPS_TOPIC = "/gps/fix"

    extract_gps_to_unix_named_file(BAG_FILE, GPS_TOPIC)