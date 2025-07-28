#!/usr/bin/env python3
import rosbag
import rospy
import sys
import os
from datetime import datetime

class RosbagSplitter:
    def __init__(self, input_bag_path, output_dir):
        self.input_bag_path = input_bag_path
        self.output_dir = output_dir
        
        # Create output directory if it doesn't exist
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
    
    def get_bag_info(self):
        """Get information about the bag file"""
        with rosbag.Bag(self.input_bag_path, 'r') as bag:
            info = {
                'start_time': bag.get_start_time(),  # Returns float timestamp
                'end_time': bag.get_end_time(),      # Returns float timestamp
                'topics': list(bag.get_type_and_topic_info()[1].keys()),
                'message_count': sum([bag.get_type_and_topic_info()[1][topic].message_count 
                                    for topic in bag.get_type_and_topic_info()[1].keys()])
            }
        return info
    
    def split_by_time(self, segment_duration_minutes=2, overlap_seconds=0):
        """
        Split the bag file by time
        
        Args:
            segment_duration_minutes: Duration of each segment in minutes
            overlap_seconds: Overlap duration between segments in seconds
        """
        info = self.get_bag_info()
        start_time = info['start_time']
        end_time = info['end_time']
        
        print(f"Starting to split: {self.input_bag_path}")
        print(f"Time range: {datetime.fromtimestamp(start_time)} - {datetime.fromtimestamp(end_time)}")
        print(f"Total duration: {(end_time - start_time)/60:.2f} minutes")
        
        segment_duration = segment_duration_minutes * 60
        overlap_duration = overlap_seconds
        
        segment_count = 0
        current_start = start_time
        
        while current_start < end_time:
            current_end = min(current_start + segment_duration, end_time)
            
            # Generate output filename
            start_dt = datetime.fromtimestamp(current_start)
            output_filename = f"segment_{segment_count:03d}_{start_dt.strftime('%Y%m%d_%H%M%S')}.bag"
            output_path = os.path.join(self.output_dir, output_filename)
            
            # Write the segment
            messages_written = self._write_segment(current_start, current_end, output_path)
            
            print(f"Created: {output_filename} (Messages: {messages_written})")
            
            segment_count += 1
            current_start = current_end - overlap_duration
            
            if current_end >= end_time:
                break
        
        print(f"Splitting complete! Created {segment_count} segments.")
    
    def _write_segment(self, start_time_float, end_time_float, output_path):
        """Write a time segment to a new bag file"""
        message_count = 0
        
        # Convert float time to rospy.Time object
        start_time = rospy.Time.from_sec(start_time_float)
        end_time = rospy.Time.from_sec(end_time_float)
        
        with rosbag.Bag(self.input_bag_path, 'r') as input_bag:
            with rosbag.Bag(output_path, 'w') as output_bag:
                try:
                    for topic, msg, t in input_bag.read_messages(start_time=start_time, end_time=end_time):
                        output_bag.write(topic, msg, t)
                        message_count += 1
                except Exception as e:
                    print(f"Warning: Error processing message: {e}")
                    # If there's an issue with time filtering, try without it
                    if message_count == 0:
                        print("Retrying...")
                        for topic, msg, t in input_bag.read_messages():
                            # Manually check time range
                            if t >= start_time and t < end_time:
                                output_bag.write(topic, msg, t)
                                message_count += 1
                            elif t >= end_time:
                                break
        return message_count

# Simplified version of the split function
def simple_split_rosbag(input_bag_path, output_dir, segment_duration_minutes=2):
    """
    Simple version of the rosbag splitting function
    """
    # Create output directory
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Get bag file info
    with rosbag.Bag(input_bag_path, 'r') as bag:
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()
    
    print(f"Bag start time: {datetime.fromtimestamp(start_time)}")
    print(f"Bag end time: {datetime.fromtimestamp(end_time)}")
    print(f"Total duration: {(end_time - start_time)/60:.2f} minutes")
    
    # Calculate split parameters
    segment_duration = segment_duration_minutes * 60
    num_segments = int((end_time - start_time) / segment_duration) + 1
    
    print(f"Splitting into {num_segments} segments, each {segment_duration_minutes} minute(s)")
    
    # Perform splitting
    for i in range(num_segments):
        segment_start = start_time + i * segment_duration
        segment_end = min(segment_start + segment_duration, end_time)
        
        # Skip if the segment is beyond the bag's end time
        if segment_start >= end_time:
            break
            
        # Generate output filename
        start_dt = datetime.fromtimestamp(segment_start)
        output_filename = f"segment_{i:03d}_{start_dt.strftime('%Y%m%d_%H%M%S')}.bag"
        output_path = os.path.join(output_dir, output_filename)
        
        # Filter and write to new bag file
        message_count = 0
        with rosbag.Bag(input_bag_path, 'r') as input_bag:
            with rosbag.Bag(output_path, 'w') as output_bag:
                try:
                    # Use float timestamps for comparison
                    for topic, msg, t in input_bag.read_messages():
                        t_float = t.to_sec()
                        if t_float >= segment_start and t_float < segment_end:
                            output_bag.write(topic, msg, t)
                            message_count += 1
                        elif t_float >= segment_end:
                            break
                except Exception as e:
                    print(f"Error processing segment {i}: {e}")
        
        print(f"Segment {i+1}/{num_segments}: {output_filename} ({message_count} messages)")
        
        # If the current segment ends at or after the bag's end time, stop processing
        if segment_end >= end_time:
            break
    
    print("Splitting completed!")

# Usage example
if __name__ == "__main__":
    # Initialize ROS node (optional but helps with time handling)
    rospy.init_node('bag_splitter', anonymous=True)
    
    if len(sys.argv) < 2:
        print("Usage:")
        print("python split_bag.py <input_bag_file> [output_directory] [duration_minutes]")
        sys.exit(1)
    
    input_bag = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else "split_bags"
    duration_minutes = int(sys.argv[3]) if len(sys.argv) > 3 else 2
    
    # Use simplified version
    simple_split_rosbag(input_bag, output_dir, duration_minutes)