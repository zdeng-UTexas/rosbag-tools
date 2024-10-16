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

            # Skip processing if images have already been extracted
            image_csv_path = os.path.join(output_dir, f"{image_topic_sanitized}_timestamps.csv")
            if os.path.exists(image_csv_path):
                logging.info(f"Images for topic {image_topic} already extracted. Skipping.")
                continue

            image_data = []
            total_messages = bag.get_message_count(topic_filters=[image_topic])
            if total_messages == 0:
                logging.info(f"No messages found for topic {image_topic}.")
                continue

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
                image_df.to_csv(image_csv_path, index=False)
                logging.info(f"Extracted {len(image_data)} images from topic {image_topic} to {output_dir}")
            else:
                logging.info(f"No images extracted from topic {image_topic}")

    logging.info(f"Image extraction from {bag_file} complete.")

def extract_gps_fix(bag_file, base_output_dir):
    fix_data = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/trevor/ublox/fix']):
            timestamp = msg.header.stamp.to_sec() if hasattr(msg, 'header') and msg.header.stamp else t.to_sec()

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

    if fix_data:
        fix_df = pd.DataFrame(fix_data)
        fix_csv_path = os.path.join(base_output_dir, 'fix_data.csv')
        fix_df.to_csv(fix_csv_path, index=False)
        logging.info(f"Saved Fix data to {fix_csv_path}")
    else:
        logging.info("No Fix data found.")

    logging.info(f"GPS Fix data extraction from {bag_file} complete.")
    return fix_data

def interpolate_gps_for_images(base_output_dir, image_folders, fix_data):
    if len(fix_data) == 0:
        logging.info("Fix data is empty, skipping interpolation.")
        return

    fix_df = pd.DataFrame(fix_data)
    fix_df['timestamp'] = fix_df['timestamp'].astype(float)
    fix_df.sort_values('timestamp', inplace=True)

    for folder in image_folders:
        image_folder_path = os.path.join(base_output_dir, folder)
        if not os.path.exists(image_folder_path):
            logging.info(f"Image folder {image_folder_path} does not exist, skipping.")
            continue

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
        timestamps_fix = fix_df['timestamp'].values
        columns_to_interpolate = ['latitude', 'longitude', 'altitude']

        for timestamp in image_timestamps:
            interpolated_values = {col: np.interp(timestamp, timestamps_fix, fix_df[col].values) for col in columns_to_interpolate}
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
        # '/trevor/multisense_rear/aux/image_color/compressed',
        # '/trevor/stereo_left/image_rect_color/compressed',
        # '/trevor/stereo_right/image_rect_color/compressed',
        # '/trevor/multisense_forward/aux/image_rect_color',
        # '/trevor/multisense_rear/aux/image_rect_color',
        # '/trevor/multisense_forward/left/image_rect',
        # '/trevor/multisense_forward/right/image_rect',
        # '/trevor/multisense_rear/left/image_rect',
        # '/trevor/multisense_rear/right/image_rect'
    ]

    # Use only the /trevor/ublox/fix topic for GPS data
    extracted_data_base_dir = '/extracted_data'

    for bag_file in tqdm(bag_files, desc="Processing bag files"):
        bag_name = os.path.splitext(os.path.basename(bag_file))[0]
        base_output_dir = os.path.join(extracted_data_base_dir, f'rosbag_{bag_name}')

        # Check if processing is already complete
        completion_marker = os.path.join(base_output_dir, 'processing_complete.txt')
        if os.path.exists(completion_marker):
            logging.info(f"Bag file {bag_file} has already been processed. Skipping.")
            continue

        os.makedirs(base_output_dir, exist_ok=True)
        logging.info(f"Processing bag file: {bag_file}")

        extract_images(bag_file, base_output_dir, image_topics)
        fix_data = extract_gps_fix(bag_file, base_output_dir)

        image_folders = [image_topic.replace('/', '_').strip('_') for image_topic in image_topics]
        interpolate_gps_for_images(base_output_dir, image_folders, fix_data)

        # Create a completion marker
        with open(completion_marker, 'w') as f:
            f.write('Processing complete.\n')
        logging.info(f"Finished processing bag file: {bag_file}")

    logging.info("Processing complete.")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logging.exception(f"An error occurred: {e}")
