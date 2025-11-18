#!/usr/bin/env python

import os
import rosbag
import cv2
import numpy as np
import csv
import argparse
from tqdm import tqdm

def extract_data(rosbag_path):
    """
    Extracts image and GPS data from a rosbag file and saves it to organized folders.

    Args:
        rosbag_path (str): The full path to the .bag file.
    """
    # 1. Setup Paths and Directories
    if not os.path.exists(rosbag_path):
        print(f"Error: Rosbag file not found at '{rosbag_path}'")
        return

    # Create a main output directory named after the rosbag file
    base_name = os.path.splitext(os.path.basename(rosbag_path))[0]
    output_dir = os.path.join(os.path.dirname(rosbag_path), base_name)
    os.makedirs(output_dir, exist_ok=True)
    print(f"Created main output directory: {output_dir}")

    # GPS file setup
    gps_dir = os.path.join(output_dir, 'gps')
    os.makedirs(gps_dir, exist_ok=True)
    gps_file_path = os.path.join(gps_dir, 'gps_data.csv')
    
    # List to hold all GPS data before writing to CSV
    gps_data_list = []
    
    # Dictionary to hold image output directories
    image_dirs = {}

    print("Processing rosbag...")
    try:
        with rosbag.Bag(rosbag_path, 'r') as bag:
            # Use tqdm for a progress bar
            total_messages = bag.get_message_count()
            pbar = tqdm(total=total_messages, unit="msgs")

            for topic, msg, t in bag.read_messages():
                # 2. Process GPS Messages
                if topic == '/gps/fix':
                    gps_data_list.append([
                        t.to_sec(),
                        msg.latitude,
                        msg.longitude,
                        msg.altitude
                    ])

                # 3. Process Image Messages
                elif '4k_image/compressed' in topic:
                    # Get camera ID from topic name (e.g., 'DA4930148')
                    camera_id = topic.split('/')[1]
                    
                    # Create a specific directory for this camera if it's the first time seeing it
                    if camera_id not in image_dirs:
                        cam_dir = os.path.join(output_dir, camera_id)
                        os.makedirs(cam_dir, exist_ok=True)
                        image_dirs[camera_id] = cam_dir
                        print(f"Created image directory for {camera_id}: {cam_dir}")

                    # Decode the compressed image
                    np_arr = np.frombuffer(msg.data, np.uint8)
                    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                    # Generate a filename from the message timestamp
                    # Using the timestamp ensures chronological order and uniqueness
                    timestamp_str = f"{t.to_sec():.6f}"
                    image_filename = f"{timestamp_str}.jpg"
                    image_save_path = os.path.join(image_dirs[camera_id], image_filename)

                    # Save the image
                    cv2.imwrite(image_save_path, image_np)

                pbar.update(1)
            pbar.close()

    except rosbag.bag.ROSBagUnindexedException:
        print("\nError: The rosbag is unindexed. Please run 'rosbag reindex <your_rosbag>.bag' first.")
        return
    except Exception as e:
        print(f"\nAn error occurred: {e}")
        return

    # 4. Save GPS data to CSV
    if gps_data_list:
        print(f"\nSaving GPS data to {gps_file_path}...")
        with open(gps_file_path, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            # Write header
            csv_writer.writerow(['timestamp', 'latitude', 'longitude', 'altitude'])
            # Write data rows
            csv_writer.writerows(gps_data_list)
        print("GPS data saved successfully.")
    else:
        print("No GPS data found in the rosbag.")

    print("\nExtraction complete! ✨")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Extract image and GPS data from a ROS bag file.")
    parser.add_argument('rosbag_file', type=str, help='The path to the .bag file.')
    args = parser.parse_args()
    
    extract_data(args.rosbag_file)