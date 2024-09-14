import os
import pandas as pd
import numpy as np

# Base output directory containing images and GPS data
base_output_dir = '/home/zd3534/phoenix-r1/bags/rosbag_2024-08-14-14-12-48'

# Load GPS data (from NavPVT or fix_data)
navpvt_path = os.path.join(base_output_dir, 'navpvt_data.csv')
navpvt_df = pd.read_csv(navpvt_path)

# Convert timestamps in GPS data to seconds
navpvt_df['timestamp'] = navpvt_df['timestamp'].astype(float)

# List of image folders
image_folders = [
    'trevor_multisense_forward_aux_image_rect_color',
    'trevor_multisense_forward_left_image_rect',
    'trevor_multisense_forward_right_image_rect',
    'trevor_multisense_rear_aux_image_rect_color',
    'trevor_multisense_rear_left_image_rect',
    'trevor_multisense_rear_right_image_rect'
]

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
    for timestamp in image_timestamps:
        # Use NumPy's interpolation to find lat/lon/height/etc.
        interp_lat = np.interp(timestamp, navpvt_df['timestamp'], navpvt_df['lat'])
        interp_lon = np.interp(timestamp, navpvt_df['timestamp'], navpvt_df['lon'])
        interp_height = np.interp(timestamp, navpvt_df['timestamp'], navpvt_df['height'])
        interp_hMSL = np.interp(timestamp, navpvt_df['timestamp'], navpvt_df['hMSL'])
        interp_hAcc = np.interp(timestamp, navpvt_df['timestamp'], navpvt_df['hAcc'])
        interp_vAcc = np.interp(timestamp, navpvt_df['timestamp'], navpvt_df['vAcc'])
        interp_velN = np.interp(timestamp, navpvt_df['timestamp'], navpvt_df['velN'])
        interp_velE = np.interp(timestamp, navpvt_df['timestamp'], navpvt_df['velE'])
        interp_velD = np.interp(timestamp, navpvt_df['timestamp'], navpvt_df['velD'])
        interp_gSpeed = np.interp(timestamp, navpvt_df['timestamp'], navpvt_df['gSpeed'])
        interp_heading = np.interp(timestamp, navpvt_df['timestamp'], navpvt_df['heading'])
        interp_sAcc = np.interp(timestamp, navpvt_df['timestamp'], navpvt_df['sAcc'])
        interp_headAcc = np.interp(timestamp, navpvt_df['timestamp'], navpvt_df['headAcc'])

        # Add interpolated data to the list
        interpolated_gps_data.append({
            'timestamp': timestamp,
            'lat': interp_lat,
            'lon': interp_lon,
            'height': interp_height,
            'hMSL': interp_hMSL,
            'hAcc': interp_hAcc,
            'vAcc': interp_vAcc,
            'velN': interp_velN,
            'velE': interp_velE,
            'velD': interp_velD,
            'gSpeed': interp_gSpeed,
            'heading': interp_heading,
            'sAcc': interp_sAcc,
            'headAcc': interp_headAcc
        })

    # Convert the list of interpolated data to a DataFrame
    interpolated_gps_df = pd.DataFrame(interpolated_gps_data)
    
    # Save the interpolated GPS data to a CSV file
    interpolated_gps_path = os.path.join(base_output_dir, f'{folder}_interpolated_gps.csv')
    interpolated_gps_df.to_csv(interpolated_gps_path, index=False)
    
    print(f"Interpolated GPS data saved to {interpolated_gps_path}")

print("Interpolation complete.")
