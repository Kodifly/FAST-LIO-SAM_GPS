#!/usr/bin/env python3

import rosbag
import argparse
import os

def extract_imu_data(bag_file, topic_name, output_file):
    """
    Extracts IMU data from a ROS bag and writes it line-by-line to a TXT file,
    including a descriptive header line and periodic progress logging.
    """
    print(f"Extracting IMU data from topic '{topic_name}' in '{bag_file}'...")
    
    LOG_INTERVAL = 1000  # Log every N messages
    message_count = 0

    with open(output_file, 'w') as txtfile:
        # Write header line
        header = "timestamp [ns] orientation_x orientation_y orientation_z orientation_w " \
                 "angular_velocity_x angular_velocity_y angular_velocity_z " \
                 "linear_acceleration_x linear_acceleration_y linear_acceleration_z\n"
        txtfile.write(header)

        bag = rosbag.Bag(bag_file, "r")
        
        try:
            for topic, msg, t in bag.read_messages(topics=[topic_name]):
                try:
                    # Use message timestamp (in nanoseconds)
                    timestamp = msg.header.stamp.to_sec()

                    # Format all values into one line
                    line = f"{timestamp} " \
                           f"{msg.orientation.x} {msg.orientation.y} {msg.orientation.z} {msg.orientation.w} " \
                           f"{msg.angular_velocity.x} {msg.angular_velocity.y} {msg.angular_velocity.z} " \
                           f"{msg.linear_acceleration.x} {msg.linear_acceleration.y} {msg.linear_acceleration.z}\n"

                    txtfile.write(line)
                    message_count += 1

                    # Periodic log
                    if message_count % LOG_INTERVAL == 0:
                        print(f"Processed {message_count} IMU messages...")

                except Exception as e:
                    print(f"Error processing IMU message: {e}")

            print(f"✅ Successfully extracted {message_count} IMU messages to {output_file}")
        finally:
            bag.close()


def find_imu_topics(bag_file):
    imu_topics = []
    with rosbag.Bag(bag_file, 'r') as bag:
        info = bag.get_type_and_topic_info()
        for topic in info.topics:
            if info.topics[topic].msg_type == 'sensor_msgs/Imu':
                imu_topics.append(topic)
    return imu_topics


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Extract IMU data from a ROS bag.')
    parser.add_argument('bag_file', help='Input ROS bag file')
    parser.add_argument('--topic', default=None, help='Specify a particular IMU topic (optional)')
    parser.add_argument('--output', default='imu_data.txt', help='Output TXT file name')

    args = parser.parse_args()

    if not os.path.exists(args.bag_file):
        print("❌ The specified bag file does not exist.")
        exit(1)

    # Find all IMU topics if none is specified
    imu_topics = find_imu_topics(args.bag_file)

    if not imu_topics:
        print("❌ No IMU topics found in the bag file.")
        exit(1)

    print("Available IMU topics:")
    for i, topic in enumerate(imu_topics):
        print(f"{i}: {topic}")

    if args.topic:
        selected_topic = args.topic
    else:
        choice = int(input("Select an IMU topic by index: "))
        selected_topic = imu_topics[choice]

    extract_imu_data(args.bag_file, selected_topic, args.output)