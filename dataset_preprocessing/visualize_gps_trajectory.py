import os
import pandas as pd
import numpy as np
from simplekml import Kml

def interpolate_gps(image_timestamps, gps_data):
    """
    Interpolates latitude and longitude for each image timestamp based on GPS data.

    Parameters:
        image_timestamps (list of float): Timestamps extracted from image filenames.
        gps_data (pd.DataFrame): DataFrame containing 'timestamp', 'latitude', and 'longitude'.

    Returns:
        tuple: Two numpy arrays containing interpolated latitudes and longitudes.
    """
    # Convert the GPS timestamp and image timestamp to float for interpolation
    gps_timestamps = gps_data['timestamp'].astype(float).values
    latitudes = gps_data['latitude'].values
    longitudes = gps_data['longitude'].values

    # Interpolate latitude and longitude for each image timestamp
    interpolated_lat = np.interp(image_timestamps, gps_timestamps, latitudes)
    interpolated_lon = np.interp(image_timestamps, gps_timestamps, longitudes)

    return interpolated_lat, interpolated_lon

def generate_kml(gps_data, kml_file_path):
    """
    Generates a KML file from GPS data.

    Parameters:
        gps_data (pd.DataFrame): DataFrame containing 'latitude', 'longitude', 'timestamp', and 'image_path'.
        kml_file_path (str): Path where the KML file will be saved.
    """
    kml = Kml()
    for _, row in gps_data.iterrows():
        lat, lon = row['latitude'], row['longitude']
        timestamp = row['timestamp']
        img_path = row['image_path']
        
        # Create a point in the KML
        point = kml.newpoint(name=f'Timestamp: {timestamp}', coords=[(lon, lat)])
        point.description = f"Image Path: {img_path}"
    
    kml.save(kml_file_path)
    print(f"KML file saved as {kml_file_path}")

def process_rosbag_folder(rosbag_folder, kml_output_folder, overview_kml):
    """
    Processes a single rosbag folder: interpolates GPS data and generates KML files.

    Parameters:
        rosbag_folder (str): Path to the rosbag folder.
        kml_output_folder (str): Directory where individual KML files are saved.
        overview_kml (simplekml.Kml): KML object for the overview file.
    """
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
    
    if not image_files:
        print(f"No image files found in {image_folder}. Skipping.")
        return
    
    # Extract timestamps from image filenames (assumes format like frame_1723481180.099995.jpg)
    try:
        image_timestamps = [float(f.split('_')[1].replace('.jpg', '')) for f in image_files]
    except (IndexError, ValueError) as e:
        print(f"Error parsing timestamps in {image_folder}: {e}. Skipping.")
        return
    
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
    
    # Generate individual KML file
    rosbag_name = os.path.basename(rosbag_folder)
    individual_kml_path = os.path.join(kml_output_folder, f'{rosbag_name}.kml')
    generate_kml(result_df, individual_kml_path)
    
    # Add points to the overview KML
    for _, row in result_df.iterrows():
        lat, lon = row['latitude'], row['longitude']
        timestamp = row['timestamp']
        img_path = row['image_path']
        
        # Create a point in the overview KML
        point = overview_kml.newpoint(name=f'{rosbag_name} - {timestamp}', coords=[(lon, lat)])
        point.description = f"Image Path: {img_path}"

def process_all_rosbag_folders(base_folder, kml_output_folder, overview_kml_path):
    """
    Processes all rosbag folders and generates individual and overview KML files.

    Parameters:
        base_folder (str): Path to the base 'extracted_data' directory.
        kml_output_folder (str): Directory where individual KML files are saved.
        overview_kml_path (str): Path where the overview KML file will be saved.
    """
    # Ensure the output folder exists
    os.makedirs(kml_output_folder, exist_ok=True)
    
    # Initialize the overview KML
    overview_kml = Kml()
    
    # Iterate through each rosbag folder
    for folder_name in os.listdir(base_folder):
        rosbag_folder = os.path.join(base_folder, folder_name)
        if os.path.isdir(rosbag_folder) and folder_name.startswith('rosbag_'):
            print(f"Processing {rosbag_folder}...")
            process_rosbag_folder(rosbag_folder, kml_output_folder, overview_kml)
    
    # Save the overview KML file
    overview_kml.save(overview_kml_path)
    print(f"Overview KML file saved as {overview_kml_path}")

if __name__ == '__main__':
    base_folder = '/robodata/ARL_SARA/GQ-dataset/extracted_data'  # Update with your base folder path
    kml_output_folder = os.path.join(base_folder, 'kml')  # Path to save individual KML files
    overview_kml_path = os.path.join(kml_output_folder, 'overview.kml')  # Path to save the overview KML file
    
    process_all_rosbag_folders(base_folder, kml_output_folder, overview_kml_path)
