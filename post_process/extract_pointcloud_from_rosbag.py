#!/usr/bin/env python3

import os
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

def save_pcd(points_array, field_names, pcd_filename):
    """
    Save point cloud data as a .pcd file in ASCII format.
    """
    with open(pcd_filename, "w") as f:
        # Write the PCD header
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write(f"FIELDS {' '.join(field_names)}\n")
        f.write(f"SIZE {' '.join(['4'] * len(field_names))}\n")  # All fields are float32 (4 bytes)
        f.write(f"TYPE {' '.join(['F'] * len(field_names))}\n")  # All fields are float
        f.write(f"COUNT {' '.join(['1'] * len(field_names))}\n")  # One value per field
        f.write(f"WIDTH {len(points_array)}\n")  # Number of points
        f.write("HEIGHT 1\n")  # Unorganized point cloud
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")  # Default viewpoint
        f.write(f"POINTS {len(points_array)}\n")  # Number of points
        f.write("DATA ascii\n")

        # Write the point cloud data
        for point in points_array:
            line = " ".join([f"{point[field]:.6f}" for field in field_names])
            f.write(line + "\n")

    print(f"Saved point cloud with all fields as .pcd: {pcd_filename}")

def save_bin(points_array, field_names, bin_filename):
    """
    Save point cloud data as a .bin file in binary format.
    """
    # Define the dtype for the structured NumPy array
    dtype = [(field, 'f4') for field in field_names]  # Assume all fields are float32 (4 bytes)

    # Create a structured NumPy array
    data = np.zeros(len(points_array), dtype=dtype)
    for field in field_names:
        data[field] = points_array[field]

    # Save the combined data as a binary file
    data.tofile(bin_filename)

    print(f"Saved point cloud with all fields as .bin: {bin_filename}")

def extract_pointcloud_from_bag(bag_file, pointcloud_topic, output_dir):
    """
    Extract point cloud data with all channels from a ROS bag file.
    Saves .pcd file (ASCII) and .bin file (binary) simultaneously.
    Filenames are in the 'sec.nsec' format, with nsec padded to 9 digits.
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    bag = rosbag.Bag(bag_file, "r")

    for topic, msg, t in bag.read_messages(topics=[pointcloud_topic]):
        try:
            # Extract seconds and nanoseconds from the timestamp
            sec = msg.header.stamp.secs
            nsec = msg.header.stamp.nsecs

            # Ensure nsec is always 9 digits by padding with leading zeros
            nsec_padded = f"{nsec:09d}"  # Formats nsec as a 9-digit number (e.g., 000000000)

            # Get all available fields in the point cloud
            field_names = [field.name for field in msg.fields]
            points_list = list(pc2.read_points(msg, skip_nans=True, field_names=field_names))

            # Convert to a structured NumPy array
            dtype = [(field, np.float32) for field in field_names]
            points_array = np.array(points_list, dtype=dtype)

            # Save as .pcd file
            # pcd_filename = os.path.join(output_dir, f"{sec}.{nsec_padded}.pcd")
            # save_pcd(points_array, field_names, pcd_filename)

            # Save as .bin file
            bin_filename = os.path.join(output_dir, f"{sec}.{nsec_padded}.bin")
            save_bin(points_array, field_names, bin_filename)

        except Exception as e:
            print(f"Error processing point cloud: {e}")

    bag.close()

if __name__ == "__main__":
    bag_file = "/media/kodifly/Extreme SSD/isds_scan_data/Kowloon/front_right_2025-05-27-10-58-08.bag"
    pointcloud_topic = "/ouster/points"
    output_dir = "/media/kodifly/Extreme SSD/isds_scan_data/sample_data_scheduled_routes_0527/Kowloon/front_right/ouster_bin/"

    extract_pointcloud_from_bag(bag_file, pointcloud_topic, output_dir)