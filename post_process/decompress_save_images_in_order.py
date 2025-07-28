#!/usr/bin/env python3

import os
import cv2
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

def decompress_images_from_bag(bag_file, image_topic, output_dir):
    """
    Decompress images from a ROS bag file and save them as sequentially numbered .jpg files.
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    bridge = CvBridge()
    bag = rosbag.Bag(bag_file, "r")

    image_counter = 0  # Counter for sequential naming

    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        try:
            # Convert CompressedImage to OpenCV format
            cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Create the filename with leading zeros (e.g., image_00001.jpg)
            image_filename = os.path.join(output_dir, f"image_{image_counter:05d}.jpg")

            # Save the image
            cv2.imwrite(image_filename, cv_image)
            print(f"Saved decompressed image: {image_filename}")

            image_counter += 1  # Increment counter

        except Exception as e:
            print(f"Error decompressing image: {e}")

    bag.close()

if __name__ == "__main__":
    bag_file = "/home/kodifly/ssd2/tunnel_data/20250713_Jordan_Central/TST-Central-video.bag"
    image_topic = "/hikrobot/image/compressed"
    output_dir = "/home/kodifly/ssd2/tunnel_data/20250713_Jordan_Central/data/TST_Central"

    decompress_images_from_bag(bag_file, image_topic, output_dir)