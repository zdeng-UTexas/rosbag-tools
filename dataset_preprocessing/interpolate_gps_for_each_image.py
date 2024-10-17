import os
import pandas as pd
import numpy as np

def interpolate_gps(image_timestamps, gps_data):
    # Convert the GPS timestamp and image timestamp to float for interpolation
    gps_timestamps = gps_data['timestamp'].astype(float).values
    latitudes = gps_data['latitude'].values
    longitudes = gps_data['longitude'].values
    
    # Interpolate latitude and longitude for each image timestamp
    interpolated_lat = np.interp(image_timestamps, gps_timestamps, latitudes)
    interpolated_lon = np.interp(image_timestamps, gps_timestamps, longitudes)
    
    return interpolated_lat, interpolated_lon

def process_rosbag_folder(rosbag_folder):
    # Define the paths
    image_folder = os.path.join(rosbag_folder, 'trevor_multisense_forward_aux_image_color_compressed')
    gps_file = os.path.join(rosbag_folder, 'fix_data.csv')
    
    # Check if fix_data.csv exists
    if not os.path.exists(gps_file):
        print(f"No GPS data found in {rosbag_folder}. Skipping.")
        return
    
    # Load GPS data
    gps_data = pd.read_csv(gps_file)
    
    # Get the list of image files
    image_files = sorted([f for f in os.listdir(image_folder) if f.endswith('.jpg')])
    
    # Extract timestamps from image filenames (assumes format like frame_1723481180.099995.jpg)
    image_timestamps = [float(f.split('_')[1].replace('.jpg', '')) for f in image_files]
    
    # Get full paths of images
    full_paths = [os.path.join(image_folder, f) for f in image_files]
    
    # Interpolate GPS coordinates for each image timestamp
    interpolated_lat, interpolated_lon = interpolate_gps(image_timestamps, gps_data)
    
    # Create the result dataframe
    result_df = pd.DataFrame({
        'timestamp': image_timestamps,
        'image_path': full_paths,
        'latitude': interpolated_lat,
        'longitude': interpolated_lon
    })
    
    # Save the dataframe to a CSV file
    output_csv = os.path.join(image_folder, 'interpolated_gps_for_each_image.csv')
    result_df.to_csv(output_csv, index=False)
    print(f"Interpolated GPS data saved to {output_csv}")

def process_all_rosbag_folders(base_folder):
    # Iterate through each rosbag folder
    for folder_name in os.listdir(base_folder):
        rosbag_folder = os.path.join(base_folder, folder_name)
        if os.path.isdir(rosbag_folder):
            process_rosbag_folder(rosbag_folder)

if __name__ == '__main__':
    base_folder = '/robodata/ARL_SARA/GQ-dataset/extracted_data'  # Update with your base folder path
    process_all_rosbag_folders(base_folder)
