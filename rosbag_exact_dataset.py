#!/usr/bin/env python

import os
import cv2
import rosbag
import numpy as np
import pandas as pd
from cv_bridge import CvBridge, CvBridgeError

def extract_images(bag_file, base_output_dir, image_topics):
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
                    # Handle CompressedImage messages
                    if 'compressed' in image_topic:
                        # Decompress the image data
                        np_arr = np.frombuffer(msg.data, np.uint8)
                        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    else:
                        # Handle regular Image messages
                        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                except CvBridgeError as e:
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

            # Save image data to CSV
            if image_data:
                image_df = pd.DataFrame(image_data)
                image_csv_path = os.path.join(output_dir, f"{image_topic_sanitized}_timestamps.csv")
                image_df.to_csv(image_csv_path, index=False)
                print(f"Extracted {len(image_data)} images from topic {image_topic} to {output_dir}")
            else:
                print(f"No images extracted from topic {image_topic}")

    print(f"Image extraction from {bag_file} complete.")


def extract_gps(bag_file, base_output_dir, gps_topics):
    # Dictionaries to hold GPS and velocity data
    navpvt_data = []
    fix_data = []
    fix_velocity_data = []

    # Open the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        # Process the GPS and velocity topics
        for topic, msg, t in bag.read_messages(topics=gps_topics):
            # Use timestamp from message header if available, otherwise use 't'
            if hasattr(msg, 'header') and msg.header.stamp:
                timestamp = msg.header.stamp.to_sec()
            else:
                timestamp = t.to_sec()

            if topic == '/trevor/ublox/navpvt':
                # Extract data from NavPVT message
                navpvt_entry = {
                    'timestamp': timestamp,
                    'lat': msg.lat * 1e-7,          # degrees
                    'lon': msg.lon * 1e-7,          # degrees
                    'height': msg.height * 1e-3,    # meters
                    'hMSL': msg.hMSL * 1e-3,        # meters
                    'hAcc': msg.hAcc * 1e-3,        # meters
                    'vAcc': msg.vAcc * 1e-3,        # meters
                    'velN': msg.velN * 1e-3,        # m/s
                    'velE': msg.velE * 1e-3,        # m/s
                    'velD': msg.velD * 1e-3,        # m/s
                    'gSpeed': msg.gSpeed * 1e-3,    # Ground speed (m/s)
                    'heading': msg.heading * 1e-5,  # degrees
                    'sAcc': msg.sAcc * 1e-3,        # Speed accuracy (m/s)
                    'headAcc': msg.headAcc * 1e-5   # Heading accuracy (degrees)
                }
                navpvt_data.append(navpvt_entry)

            elif topic == '/trevor/ublox/fix':
                # Extract data from NavSatFix message
                fix_entry = {
                    'timestamp': timestamp,
                    'latitude': msg.latitude,        # degrees
                    'longitude': msg.longitude,      # degrees
                    'altitude': msg.altitude,        # meters
                    'position_covariance': msg.position_covariance,
                    'status': msg.status.status,
                    'service': msg.status.service
                }
                fix_data.append(fix_entry)

            elif topic == '/trevor/ublox/fix_velocity':
                # Extract data from TwistWithCovarianceStamped message
                fix_velocity_entry = {
                    'timestamp': timestamp,
                    'linear_x': msg.twist.twist.linear.x,
                    'linear_y': msg.twist.twist.linear.y,
                    'linear_z': msg.twist.twist.linear.z,
                    'angular_x': msg.twist.twist.angular.x,
                    'angular_y': msg.twist.twist.angular.y,
                    'angular_z': msg.twist.twist.angular.z,
                    'covariance': msg.twist.covariance
                }
                fix_velocity_data.append(fix_velocity_entry)

    # After processing, save GPS and velocity data to CSV files
    # Save NavPVT data if any
    if navpvt_data:
        navpvt_df = pd.DataFrame(navpvt_data)
        navpvt_csv_path = os.path.join(base_output_dir, 'navpvt_data.csv')
        navpvt_df.to_csv(navpvt_csv_path, index=False)
        print(f"Saved NavPVT data to {navpvt_csv_path}")
    else:
        print("No NavPVT data found.")

    # Save Fix data if any
    if fix_data:
        fix_df = pd.DataFrame(fix_data)
        fix_csv_path = os.path.join(base_output_dir, 'fix_data.csv')
        fix_df.to_csv(fix_csv_path, index=False)
        print(f"Saved Fix data to {fix_csv_path}")
    else:
        print("No Fix data found.")

    # Save Fix Velocity data if any
    if fix_velocity_data:
        fix_velocity_df = pd.DataFrame(fix_velocity_data)
        fix_velocity_csv_path = os.path.join(base_output_dir, 'fix_velocity_data.csv')
        fix_velocity_df.to_csv(fix_velocity_csv_path, index=False)
        print(f"Saved Fix Velocity data to {fix_velocity_csv_path}")
    else:
        print("No Fix Velocity data found.")

    print(f"GPS data extraction from {bag_file} complete.")

def interpolate_gps_for_images(base_output_dir, image_folders, navpvt_csv_path):
    # Load GPS data (from NavPVT)
    navpvt_df = pd.read_csv(navpvt_csv_path)

    # Convert timestamps in GPS data to seconds
    navpvt_df['timestamp'] = navpvt_df['timestamp'].astype(float)

    # Ensure GPS data is sorted by timestamp
    navpvt_df.sort_values('timestamp', inplace=True)

    # Iterate through image folders and interpolate GPS data
    for folder in image_folders:
        image_folder_path = os.path.join(base_output_dir, folder)

        # Get list of image files (only .jpg files)
        image_files = sorted([f for f in os.listdir(image_folder_path) if f.endswith('.jpg')])

        # Initialize an empty list for storing image timestamps
        image_timestamps = []

        # Extract timestamps from filenames
        for f in image_files:
            try:
                # We expect the filenames to have a timestamp between "frame_" and ".jpg"
                # For example: "frame_1692117168.123456.jpg"
                timestamp_str = f.split('frame_')[1].split('.jpg')[0]
                image_timestamps.append(float(timestamp_str))
            except (IndexError, ValueError) as e:
                print(f"Skipping file {f} due to incorrect format: {e}")

        if len(image_timestamps) == 0:
            print(f"No valid image timestamps found in {folder}, skipping interpolation.")
            continue

        # Create an empty list to hold interpolated GPS data
        interpolated_gps_data = []

        # Interpolate GPS data for each image timestamp
        timestamps_navpvt = navpvt_df['timestamp'].values

        # For each column to interpolate
        columns_to_interpolate = ['lat', 'lon', 'height', 'hMSL', 'hAcc', 'vAcc',
                                  'velN', 'velE', 'velD', 'gSpeed', 'heading', 'sAcc', 'headAcc']

        for timestamp in image_timestamps:
            interpolated_values = {}
            for col in columns_to_interpolate:
                interpolated_values[col] = np.interp(timestamp, timestamps_navpvt, navpvt_df[col].values)
            # Add interpolated data to the list
            interpolated_data_entry = {'timestamp': timestamp}
            interpolated_data_entry.update(interpolated_values)
            interpolated_gps_data.append(interpolated_data_entry)

        # Convert the list of interpolated data to a DataFrame
        interpolated_gps_df = pd.DataFrame(interpolated_gps_data)

        # Save the interpolated GPS data to a CSV file in the image folder
        interpolated_gps_path = os.path.join(image_folder_path, 'interpolated_gps.csv')
        interpolated_gps_df.to_csv(interpolated_gps_path, index=False)

        print(f"Interpolated GPS data saved to {interpolated_gps_path}")

    print("Interpolation complete.")

def main():
    # List of bag files
    bag_files = [
        '/home/zd3534/phoenix-r1/bags/2024-08-14-14-12-48.bag',
        '/home/zd3534/phoenix-r1/bags/2024-08-14-14-25-35.bag',
        '/home/zd3534/phoenix-r1/bags/2024-08-14-13-35-39.bag'
    ]

    # Image topics
    image_topics = [
        '/trevor/multisense_forward/aux/image_color/compressed',
        '/trevor/multisense_rear/aux/image_color/compressed',
        '/trevor/stereo_left/image_rect_color/compressed',
        '/trevor/stereo_right/image_rect_color/compressed',
        # '/trevor/multisense_forward/aux/image_rect_color',
        # '/trevor/multisense_rear/aux/image_rect_color',
        '/trevor/multisense_forward/left/image_rect',
        '/trevor/multisense_forward/right/image_rect',
        '/trevor/multisense_rear/left/image_rect',
        '/trevor/multisense_rear/right/image_rect'
    ]

    # GPS topics
    gps_topics = [
        '/trevor/ublox/navpvt',
        '/trevor/ublox/fix',
        '/trevor/ublox/fix_velocity'
    ]

    # For each bag file
    for bag_file in bag_files:
        # Extract the bag file name
        bag_name = os.path.splitext(os.path.basename(bag_file))[0]

        # Base output directory
        base_output_dir = os.path.join(os.path.dirname(bag_file), f'rosbag_{bag_name}')

        # Create the base output directory if it doesn't exist
        if not os.path.exists(base_output_dir):
            os.makedirs(base_output_dir)

        print(f"Processing bag file: {bag_file}")

        # Extract images
        extract_images(bag_file, base_output_dir, image_topics)

        # Extract GPS data
        extract_gps(bag_file, base_output_dir, gps_topics)

        # Interpolate GPS data for images
        # Need to define image folders based on extracted image topics
        # The image folders are subdirectories in base_output_dir
        # We can get the image folder names from the image topics
        image_folders = []
        for image_topic in image_topics:
            image_topic_sanitized = image_topic.replace('/', '_').strip('_')
            image_folder = image_topic_sanitized
            image_folders.append(image_folder)

        navpvt_csv_path = os.path.join(base_output_dir, 'navpvt_data.csv')
        if os.path.exists(navpvt_csv_path):
            interpolate_gps_for_images(base_output_dir, image_folders, navpvt_csv_path)
        else:
            print(f"NavPVT CSV data not found at {navpvt_csv_path}, skipping interpolation.")

    print("Processing complete.")

if __name__ == "__main__":
    main()
