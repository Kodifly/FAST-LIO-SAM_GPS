import rosbag
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

# Define the input and output bag file paths
input_bag_path = '/media/kodifly/Extreme SSD/isds_0915/2025-09-15-17-24-04.bag'
output_bag_path = '/media/kodifly/Extreme SSD/isds_0915/2025-09-15-17-24-04_modified.bag'
image_topic = '/DA4930148/4k_image/compressed'

print(f"Reading from {input_bag_path} and writing to {output_bag_path}...")

with rosbag.Bag(output_bag_path, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(input_bag_path, 'r').read_messages():
        
        # Check if the message is the compressed image we want to modify
        if topic == image_topic and msg._type == 'sensor_msgs/CompressedImage':
            try:
                # 1. Decompress the image from the message data
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if cv_image is None:
                    print(f"Failed to decompress image at time {t.to_sec()}")
                    continue

                # Get the image dimensions
                height, width, channels = cv_image.shape
                
                # 2. Create 4 rows of black pixels (zeros)
                black_rows = np.zeros((4, width, channels), dtype=np.uint8)

                # 3. Stack the black rows onto the top of the image
                modified_image = np.vstack((black_rows, cv_image))
                
                # 4. Re-compress the modified image
                # The format of the original image is in msg.format (e.g., 'jpeg')
                # Use a matching file extension for cv2.imencode
                file_extension = '.' + msg.format.split(';')[0].strip()
                
                _, compressed_data = cv2.imencode(file_extension, modified_image)

                # 5. Create a new CompressedImage message
                modified_msg = CompressedImage()
                modified_msg.header = msg.header
                modified_msg.format = msg.format
                modified_msg.data = compressed_data.tobytes()

                # 6. Write the new message to the output bag
                outbag.write(topic, modified_msg, t)
            
            except Exception as e:
                print(f"Error processing image message: {e}")
        
        # For all other messages, just copy them to the new bag
        else:
            outbag.write(topic, msg, t)

print("Modification complete. A new rosbag has been created.")