#!/usr/bin/env python

import os
import cv2
import glob
import rosbag
import numpy as np
import pandas as pd
from cv_bridge import CvBridge, CvBridgeError
import logging
from tqdm import tqdm

# Configure logging
logging.basicConfig(
    filename='/extracted_data/extraction.log',
    filemode='a',
    format='%(asctime)s %(levelname)s:%(message)s',
    level=logging.INFO
)

def extract_images(bag_file, base_output_dir, image_topics):
    bridge = CvBridge()

    with rosbag.Bag(bag_file, 'r') as bag:
        for image_topic in image_topics:
            image_topic_sanitized = image_topic.replace('/', '_').strip('_')
            output_dir = os.path.join(base_output_dir, image_topic_sanitized)
            os.makedirs(output_dir, exist_ok=True)

            image_data = []
            total_messages = bag.get_message_count(topic_filters=[image_topic])

            for topic, msg, t in tqdm(bag.read_messages(topics=[image_topic]), total=total_messages, desc=f"Extracting {image_topic_sanitized}"):
                try:
                    # Handle CompressedImage messages
                    if 'compressed' in image_topic:
                        np_arr = np.frombuffer(msg.data, np.uint8)
                        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    else:
                        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                except CvBridgeError as e:
                    logging.error(f"Error converting image at time {t.to_sec()} on topic {image_topic}: {e}")
                    continue

                # Use timestamp from message header if available
                timestamp = msg.header.stamp.to_sec() if hasattr(msg, 'header') and msg.header.stamp else t.to_sec()

                # Save image
                img_name = os.path.join(output_dir, f"frame_{timestamp:.6f}.jpg")
                cv2.imwrite(img_name, cv_img)

                # Store image data
                image_data.append({'timestamp': timestamp, 'filename': img_name})

            # Save image data to CSV
            if image_data:
                image_df = pd.DataFrame(image_data)
                image_csv_path = os.path.join(output_dir, f"{image_topic_sanitized}_timestamps.csv")
                image_df.to_csv(image_csv_path, index=False)
                logging.info(f"Extracted {len(image_data)} images from topic {image_topic} to {output_dir}")
            else:
                logging.info(f"No images extracted from topic {image_topic}")

    logging.info(f"Image extraction from {bag_file} complete.")

def extract_gps(bag_file, base_output_dir, gps_topics):
    navpvt_data = []
    fix_data = []
    fix_velocity_data = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=gps_topics):
            timestamp = msg.header.stamp.to_sec() if hasattr(msg, 'header') and msg.header.stamp else t.to_sec()

            if topic == '/trevor/ublox/navpvt':
                navpvt_entry = {
                    'timestamp': timestamp,
                    'lat': msg.lat * 1e-7,
                    'lon': msg.lon * 1e-7,
                    'height': msg.height * 1e-3,
                    'hMSL': msg.hMSL * 1e-3,
                    'hAcc': msg.hAcc * 1e-3,
                    'vAcc': msg.vAcc * 1e-3,
                    'velN': msg.velN * 1e-3,
                    'velE': msg.velE * 1e-3,
                    'velD': msg.velD * 1e-3,
                    'gSpeed': msg.gSpeed * 1e-3,
                    'heading': msg.heading * 1e-5,
                    'sAcc': msg.sAcc * 1e-3,
                    'headAcc': msg.headAcc * 1e-5
                }
                navpvt_data.append(navpvt_entry)
            elif topic == '/trevor/ublox/fix':
                fix_entry = {
                    'timestamp': timestamp,
                    'latitude': msg.latitude,
                    'longitude': msg.longitude,
                    'altitude': msg.altitude,
                    'position_covariance': msg.position_covariance,
                    'status': msg.status.status,
                    'service': msg.status.service
                }
                fix_data.append(fix_entry)
            elif topic == '/trevor/ublox/fix_velocity':
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

    if navpvt_data:
        navpvt_df = pd.DataFrame(navpvt_data)
        navpvt_csv_path = os.path.join(base_output_dir, 'navpvt_data.csv')
        navpvt_df.to_csv(navpvt_csv_path, index=False)
        logging.info(f"Saved NavPVT data to {navpvt_csv_path}")
    else:
        logging.info("No NavPVT data found.")

    if fix_data:
        fix_df = pd.DataFrame(fix_data)
        fix_csv_path = os.path.join(base_output_dir, 'fix_data.csv')
        fix_df.to_csv(fix_csv_path, index=False)
        logging.info(f"Saved Fix data to {fix_csv_path}")
    else:
        logging.info("No Fix data found.")

    if fix_velocity_data:
        fix_velocity_df = pd.DataFrame(fix_velocity_data)
        fix_velocity_csv_path = os.path.join(base_output_dir, 'fix_velocity_data.csv')
        fix_velocity_df.to_csv(fix_velocity_csv_path, index=False)
        logging.info(f"Saved Fix Velocity data to {fix_velocity_csv_path}")
    else:
        logging.info("No Fix Velocity data found.")

    logging.info(f"GPS data extraction from {bag_file} complete.")

def interpolate_gps_for_images(base_output_dir, image_folders, navpvt_csv_path):
    navpvt_df = pd.read_csv(navpvt_csv_path)
    navpvt_df['timestamp'] = navpvt_df['timestamp'].astype(float)
    navpvt_df.sort_values('timestamp', inplace=True)

    for folder in image_folders:
        image_folder_path = os.path.join(base_output_dir, folder)
        image_files = sorted([f for f in os.listdir(image_folder_path) if f.endswith('.jpg')])

        image_timestamps = []
        for f in image_files:
            try:
                timestamp_str = f.split('frame_')[1].split('.jpg')[0]
                image_timestamps.append(float(timestamp_str))
            except (IndexError, ValueError) as e:
                logging.error(f"Skipping file {f} due to incorrect format: {e}")

        if len(image_timestamps) == 0:
            logging.info(f"No valid image timestamps found in {folder}, skipping interpolation.")
            continue

        interpolated_gps_data = []
        timestamps_navpvt = navpvt_df['timestamp'].values
        columns_to_interpolate = ['lat', 'lon', 'height', 'hMSL', 'hAcc', 'vAcc', 'velN', 'velE', 'velD', 'gSpeed', 'heading', 'sAcc', 'headAcc']

        for timestamp in image_timestamps:
            interpolated_values = {col: np.interp(timestamp, timestamps_navpvt, navpvt_df[col].values) for col in columns_to_interpolate}
            interpolated_data_entry = {'timestamp': timestamp}
            interpolated_data_entry.update(interpolated_values)
            interpolated_gps_data.append(interpolated_data_entry)

        interpolated_gps_df = pd.DataFrame(interpolated_gps_data)
        interpolated_gps_path = os.path.join(image_folder_path, 'interpolated_gps.csv')
        interpolated_gps_df.to_csv(interpolated_gps_path, index=False)
        logging.info(f"Interpolated GPS data saved to {interpolated_gps_path}")

    logging.info("Interpolation complete.")

def main():
    bag_files_dir = '/rosbags'
    bag_files = glob.glob(os.path.join(bag_files_dir, '*.bag'))
    specific_bag_files = []
    bag_files.extend(specific_bag_files)

    image_topics = [
        '/trevor/multisense_forward/aux/image_color/compressed',
        '/trevor/multisense_rear/aux/image_color/compressed',
        '/trevor/stereo_left/image_rect_color/compressed',
        '/trevor/stereo_right/image_rect_color/compressed',
        '/trevor/multisense_forward/aux/image_rect_color',
        '/trevor/multisense_rear/aux/image_rect_color',
        '/trevor/multisense_forward/left/image_rect',
        '/trevor/multisense_forward/right/image_rect',
        '/trevor/multisense_rear/left/image_rect',
        '/trevor/multisense_rear/right/image_rect'
    ]

    gps_topics = [
        '/trevor/ublox/navpvt',
        '/trevor/ublox/fix',
        '/trevor/ublox/fix_velocity'
    ]

    extracted_data_base_dir = '/extracted_data'
    
    for bag_file in tqdm(bag_files, desc="Processing bag files"):
        bag_name = os.path.splitext(os.path.basename(bag_file))[0]
        base_output_dir = os.path.join(extracted_data_base_dir, f'rosbag_{bag_name}')
        os.makedirs(base_output_dir, exist_ok=True)

        logging.info(f"Processing bag file: {bag_file}")

        extract_images(bag_file, base_output_dir, image_topics)
        extract_gps(bag_file, base_output_dir, gps_topics)

        image_folders = [image_topic.replace('/', '_').strip('_') for image_topic in image_topics]
        navpvt_csv_path = os.path.join(base_output_dir, 'navpvt_data.csv')

        if os.path.exists(navpvt_csv_path):
            interpolate_gps_for_images(base_output_dir, image_folders, navpvt_csv_path)
        else:
            logging.info(f"NavPVT CSV data not found at {navpvt_csv_path}, skipping interpolation.")

    logging.info("Processing complete.")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logging.exception(f"An error occurred: {e}")
