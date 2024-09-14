#!/usr/bin/env python

import rosbag
import cv2
from cv_bridge import CvBridge
import os
import pandas as pd

# Input parameters
bag_file = '/home/zd3534/phoenix-r1/bags/2024-08-14-14-12-48.bag'
image_topics = [
    '/trevor/multisense_forward/aux/image_rect_color',
    '/trevor/multisense_forward/left/image_rect',
    '/trevor/multisense_forward/right/image_rect',
    '/trevor/multisense_rear/aux/image_rect_color',
    '/trevor/multisense_rear/left/image_rect',
    '/trevor/multisense_rear/right/image_rect'
]

# Extract the bag file name
bag_name = os.path.splitext(os.path.basename(bag_file))[0]

# Base output directory
base_output_dir = os.path.join(os.path.dirname(bag_file), f'rosbag_{bag_name}')

# Create a CvBridge instance
bridge = CvBridge()

# Open the bag file
with rosbag.Bag(bag_file, 'r') as bag:
    # Loop over each image topic
    for image_topic in image_topics:
        # Sanitize the image topic to create a valid directory name
        image_topic_sanitized = image_topic.replace('/', '_').strip('_')
        output_dir = os.path.join(base_output_dir, image_topic_sanitized)

        # Create the output directory if it doesn't exist
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # Initialize a list to store image data (filenames and timestamps)
        image_data = []

        # Read messages from the current image topic
        for topic, msg, t in bag.read_messages(topics=[image_topic]):
            try:
                # Convert ROS Image message to OpenCV image
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                print(f"Error converting image at time {t.to_sec()} on topic {image_topic}: {e}")
                continue

            # Use timestamp from message header if available, else use 't'
            if hasattr(msg, 'header') and msg.header.stamp:
                timestamp = msg.header.stamp.to_sec()
            else:
                timestamp = t.to_sec()

            # Construct image file name with timestamp
            img_name = os.path.join(output_dir, f"frame_{timestamp:.6f}.jpg")
            # Save image
            cv2.imwrite(img_name, cv_img)
            # Store image data
            image_data.append({'timestamp': timestamp, 'filename': img_name})
            # print(f"Saved image {img_name}")

        # Save image data to CSV
        if image_data:
            image_df = pd.DataFrame(image_data)
            image_csv_path = os.path.join(output_dir, f"{image_topic_sanitized}_timestamps.csv")
            image_df.to_csv(image_csv_path, index=False)
            # print(f"Saved image timestamps to {image_csv_path}")
            print(f"Extracted {len(image_data)} images from topic {image_topic} to {output_dir}")
        else:
            print(f"No images extracted from topic {image_topic}")

print("Image extraction complete.")
