#!/usr/bin/env python

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

def extract_images():
    # --- Set your parameters here ---
    bag_file = "/home/kodifly/ssd2/indo_scan_data/20250710/route1.bag"  # <-- Change this
    topic_name = "/left_camera/image"            # <-- Change this
    output_dir = "/home/kodifly/ssd2/indo_scan_data/20250710/data/image_route1"       # <-- Change this
    # --------------------------------

    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    bridge = CvBridge()
    count = 0

    print(f"Opening bag file: {bag_file}")
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            try:
                # Convert ROS Image message to OpenCV image
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                print(f"Error converting image: {e}")
                continue

            # Extract seconds and nanoseconds from the timestamp
            sec = msg.header.stamp.secs
            nsec = msg.header.stamp.nsecs

            # Ensure nsec is always 9 digits by padding with leading zeros
            nsec_padded = f"{nsec:09d}"  # Formats nsec as a 9-digit number (e.g., 000000000)

            # Create the filename in 'sec.nsec.jpg' format
            filename = os.path.join(output_dir, f"{sec}.{nsec_padded}.jpg")

            # Save the image
            cv2.imwrite(filename, cv_image)
            count += 1

            if count % 100 == 0:
                print(f"Saved {count} images...")

    print(f"Done. Saved {count} images to {output_dir}")

if __name__ == "__main__":
    rospy.init_node('extract_images_node', anonymous=True)
    extract_images()