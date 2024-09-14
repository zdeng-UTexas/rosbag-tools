import os
import pandas as pd

# Base input directory containing CSV files
base_input_dir = '/home/zd3534/phoenix-r1/bags/rosbag_2024-08-14-14-12-48'

# Base output directory for KML files
base_output_dir = os.path.join(base_input_dir, 'interpolated_gps_kml')

# Create the output directory if it doesn't exist
if not os.path.exists(base_output_dir):
    os.makedirs(base_output_dir)

# Function to convert CSV to KML
def csv_to_kml(csv_file, kml_file):
    # Read CSV data
    df = pd.read_csv(csv_file)

    # Create the KML structure
    kml_header = '''<?xml version="1.0" encoding="UTF-8"?>
    <kml xmlns="http://www.opengis.net/kml/2.2">
    <Document>
        <name>Interpolated GPS Coordinates</name>
        <description>GPS points interpolated from robot data</description>
        <Style id="yellowLineGreenPoly">
            <LineStyle>
                <color>7f00ffff</color>
                <width>4</width>
            </LineStyle>
            <PolyStyle>
                <color>7f00ff00</color>
            </PolyStyle>
        </Style>
        <Placemark>
            <name>Path</name>
            <description>Interpolated GPS Path</description>
            <styleUrl>#yellowLineGreenPoly</styleUrl>
            <LineString>
                <tessellate>1</tessellate>
                <coordinates>
    '''

    kml_footer = '''            </coordinates>
            </LineString>
        </Placemark>
    </Document>
    </kml>
    '''

    # Generate coordinates in KML format (longitude, latitude, height)
    coordinates = ""
    for index, row in df.iterrows():
        coordinates += f"{row['lon']},{row['lat']},{row['height']}\n"

    # Write the KML file
    with open(kml_file, 'w') as f:
        f.write(kml_header)
        f.write(coordinates)
        f.write(kml_footer)

    print(f"KML file saved to {kml_file}")

# List of CSV files that need to be processed
csv_files = [f for f in os.listdir(base_input_dir) if f.endswith('_interpolated_gps.csv')]

# Loop through each CSV file and convert to KML
for csv_file in csv_files:
    # Full path to the CSV file
    csv_file_path = os.path.join(base_input_dir, csv_file)
    
    # Create the corresponding KML file name
    kml_file_name = csv_file.replace('_interpolated_gps.csv', '_interpolated_gps.kml')
    kml_file_path = os.path.join(base_output_dir, kml_file_name)
    
    # Convert the CSV to KML
    csv_to_kml(csv_file_path, kml_file_path)

print("Batch KML conversion complete.")
