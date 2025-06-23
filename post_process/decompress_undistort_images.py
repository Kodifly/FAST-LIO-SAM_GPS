#!/usr/bin/env python3

import os
import cv2
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import numpy as np

def process_bag_and_undistort(bag_file, image_topic, output_dir,
                             camera_matrix, dist_coeffs):
    """
    Reads compressed images from a ROS bag, undistorts them using
    the 8-parameter distortion model, and saves only the undistorted images.
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    bridge = CvBridge()
    bag = rosbag.Bag(bag_file, "r")

    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        try:
            # Convert CompressedImage to OpenCV format
            cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Undistort image
            undistorted_image = cv2.undistort(cv_image, camera_matrix, dist_coeffs,
                                             None, camera_matrix)

            # Extract timestamp
            sec = msg.header.stamp.secs
            nsec = msg.header.stamp.nsecs
            nsec_padded = f"{nsec:09d}"

            # Save undistorted image directly
            filename = os.path.join(output_dir, f"{sec}.{nsec_padded}.jpg")
            cv2.imwrite(filename, undistorted_image)
            print(f"Saved undistorted image: {filename}")

        except Exception as e:
            print(f"Error processing image: {e}")

    bag.close()

if __name__ == "__main__":
    # Paths
    BAG_FILE = "/home/kodifly/ssd3/tmp_data/2025-06-06-18-07-11.bag"
    IMAGE_TOPIC = "/left_camera/image/compressed"
    OUTPUT_DIR = "/home/kodifly/ssd3/tmp_data/img"

    # Camera parameters (8-parameter model)
    # CAMERA_MATRIX = np.array([  # wooden board
    #     [1419.488223, 0, 2091.72389],
    #     [0, 1419.783836, 1231.71474],
    #     [0, 0, 1]
    # ])

    CAMERA_MATRIX = np.array([
        [1418.723501, 0, 2016.266075],
        [0, 1418.745041, 1237.536737],
        [0, 0, 1]
    ])

    DIST_COEFFS = np.array([0.6008030073, 0.05105767781, 1.91756943e-05, 3.502701816e-06,
                            0.0003079906229, 0.935219043, 0.1711876097, 0.00430346178])

    # Run the full pipeline
    print("Starting decompression and undistortion...")
    process_bag_and_undistort(BAG_FILE, IMAGE_TOPIC, OUTPUT_DIR, CAMERA_MATRIX, DIST_COEFFS)
    print("Processing complete.")

    # 1418.723501, 1418.745041, 2016.266075, 1237.536737, 0.6008030073, 0.05105767781, 1.91756943e-05, 3.502701816e-06, 0.0003079906229, 0.935219043, 0.1711876097, 0.00430346178