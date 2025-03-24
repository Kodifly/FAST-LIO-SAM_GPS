#!/usr/bin/env python3

import os
import cv2
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

def decompress_images_from_bag(bag_file, image_topic, output_dir):
    """
    Decompress images from a ROS bag file and save them as .jpg files.
    Filenames are in the 'sec.nsec' format, with nsec padded to 9 digits.
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    bridge = CvBridge()
    bag = rosbag.Bag(bag_file, "r")

    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        try:
            # Convert CompressedImage to OpenCV format
            cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Extract seconds and nanoseconds from the timestamp
            sec = msg.header.stamp.secs
            nsec = msg.header.stamp.nsecs

            # Ensure nsec is always 9 digits by padding with leading zeros
            nsec_padded = f"{nsec:09d}"  # Formats nsec as a 9-digit number (e.g., 000000000)

            # Create the filename in 'sec.nsec.jpg' format
            image_filename = os.path.join(output_dir, f"{sec}.{nsec_padded}.jpg")

            # Save the image
            cv2.imwrite(image_filename, cv_image)
            print(f"Saved decompressed image: {image_filename}")

        except Exception as e:
            print(f"Error decompressing image: {e}")

    bag.close()

if __name__ == "__main__":
    bag_file = "/home/kodifly/datasets/rosbag/sample_data/4km_50kmh_ouster_fisheye.bag"
    image_topic = "/hik_camera/image/compressed"
    output_dir = "/home/kodifly/datasets/rosbag/sample_data/4km_50kmh_ouster_fisheye/fisheye_image/"

    decompress_images_from_bag(bag_file, image_topic, output_dir)