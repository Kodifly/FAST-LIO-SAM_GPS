#!/usr/bin/env python3

import rosbag
import os

def extract_gps_data(bag_file, topic_name, output_file):
    """
    Extracts GPS data from a ROS bag and writes it to a text file.

    Format:
        #timestamp latitude longitude altitude
    """
    with open(output_file, 'w') as f:
        # Write header if needed
        f.write("#timestamp latitude longitude altitude\n")

        bag = rosbag.Bag(bag_file, "r")
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            try:
                # Use message timestamp in seconds
                timestamp = msg.header.stamp.to_sec()
                lat = msg.latitude
                lon = msg.longitude
                alt = round(msg.altitude, 3)  # Keep only 3 decimal places for altitude

                # Write line to file
                f.write(f"{timestamp} {lat} {lon} {alt}\n")
            except Exception as e:
                print(f"Error processing GPS message: {e}")

        bag.close()
    print(f"GPS data saved to: {output_file}")

if __name__ == "__main__":
    # Input ROS bag file
    BAG_FILE = "/home/kodifly/ssd3/isds_scan_data_0618/2025-06-18-14-17-23.bag"

    # GPS topic name
    GPS_TOPIC = "/gps/fix"

    # Output file path
    OUTPUT_FILE = "/home/kodifly/ssd3/isds_scan_data_0618/kowloon/gps_data.txt"

    # Run extraction
    extract_gps_data(BAG_FILE, GPS_TOPIC, OUTPUT_FILE)