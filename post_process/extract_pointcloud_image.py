#!/usr/bin/env python3

import os
import cv2
import rosbag
import numpy as np
import sensor_msgs.point_cloud2 as pc2

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, PointCloud2


def save_bin(points_array, field_names, bin_filename):
    """
    Save point cloud data as a .bin file in binary format.
    """
    dtype = [(field, 'f4') for field in field_names]
    data = np.zeros(len(points_array), dtype=dtype)
    for field in field_names:
        data[field] = points_array[field]

    data.tofile(bin_filename)
    print(f"Saved point cloud as .bin: {bin_filename}")


def undistort_image(cv_image, camera_matrix, dist_coeffs):
    """Undistort an image using 8-parameter model."""
    return cv2.undistort(cv_image, camera_matrix, dist_coeffs, None, camera_matrix)


if __name__ == "__main__":
    BAG_FILE = "/home/kodifly/ssd3/isds_scan_data_0618/2025-06-18-14-17-23.bag"

    # Topics
    POINTCLOUD_TOPIC = "/ouster/points"
    IMAGE_TOPIC = "/DA4930148/image/compressed"

    # Output directories
    PC_OUTPUT_DIR = "/home/kodifly/ssd3/isds_scan_data_0618/kowloon/DA4930148/ouster_bin/"
    IMG_OUTPUT_DIR = "/home/kodifly/ssd3/isds_scan_data_0618/kowloon/DA4930148/rectificed_image/"

    # Camera parameters (8-parameter distortion model)
    CAMERA_MATRIX = np.array([
        [1419.488223, 0, 2091.72389],
        [0, 1419.783836, 1231.71474],
        [0, 0, 1]
    ])

    DIST_COEFFS = np.array([0.4765817545, 0.0267510319, 1.495544387e-07, -2.581193063e-07,
                            8.833337814e-05, 0.8074516265, 0.1103335164, 0.001695886702])

    # Create output directories if not exist
    os.makedirs(PC_OUTPUT_DIR, exist_ok=True)
    os.makedirs(IMG_OUTPUT_DIR, exist_ok=True)

    # Initialize CV Bridge
    bridge = CvBridge()

    print("Reading bag file once and processing both topics...")
    bag = rosbag.Bag(BAG_FILE, "r")

    for topic, msg, t in bag.read_messages(topics=[POINTCLOUD_TOPIC, IMAGE_TOPIC]):
        try:
            if topic == POINTCLOUD_TOPIC:
                # --- Process Point Cloud ---
                sec = msg.header.stamp.secs
                nsec = msg.header.stamp.nsecs
                nsec_padded = f"{nsec:09d}"

                field_names = [field.name for field in msg.fields]
                points_list = list(pc2.read_points(msg, skip_nans=True, field_names=field_names))

                if not points_list:
                    continue

                dtype = [(field, np.float32) for field in field_names]
                points_array = np.array(points_list, dtype=dtype)

                bin_filename = os.path.join(PC_OUTPUT_DIR, f"{sec}.{nsec_padded}.bin")
                save_bin(points_array, field_names, bin_filename)

            elif topic == IMAGE_TOPIC:
                # --- Process Image ---
                sec = msg.header.stamp.secs
                nsec = msg.header.stamp.nsecs
                nsec_padded = f"{nsec:09d}"

                cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
                undistorted_image = undistort_image(cv_image, CAMERA_MATRIX, DIST_COEFFS)

                filename = os.path.join(IMG_OUTPUT_DIR, f"{sec}.{nsec_padded}.jpg")
                cv2.imwrite(filename, undistorted_image)
                print(f"Saved undistorted image: {filename}")

        except Exception as e:
            print(f"Error processing message on topic {topic}: {e}")

    bag.close()
    print("✅ Done. All data processed in a single pass.")