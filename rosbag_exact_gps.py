#!/usr/bin/env python

import rosbag
import os
import pandas as pd

# Input parameters
bag_file = '/home/zd3534/phoenix-r1/bags/2024-08-14-14-12-48.bag'

gps_topics = [
    '/trevor/ublox/navpvt',
    '/trevor/ublox/fix',
    '/trevor/ublox/fix_velocity'
]

# Extract the bag file name
bag_name = os.path.splitext(os.path.basename(bag_file))[0]

# Base output directory
base_output_dir = os.path.join(os.path.dirname(bag_file), f'rosbag_{bag_name}')

# Create output directories if they don't exist
if not os.path.exists(base_output_dir):
    os.makedirs(base_output_dir)

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
# Save NavPVT data
navpvt_df = pd.DataFrame(navpvt_data)
navpvt_csv_path = os.path.join(base_output_dir, 'navpvt_data.csv')
navpvt_df.to_csv(navpvt_csv_path, index=False)
print(f"Saved NavPVT data to {navpvt_csv_path}")

# Save Fix data
fix_df = pd.DataFrame(fix_data)
fix_csv_path = os.path.join(base_output_dir, 'fix_data.csv')
fix_df.to_csv(fix_csv_path, index=False)
print(f"Saved Fix data to {fix_csv_path}")

# Save Fix Velocity data
fix_velocity_df = pd.DataFrame(fix_velocity_data)
fix_velocity_csv_path = os.path.join(base_output_dir, 'fix_velocity_data.csv')
fix_velocity_df.to_csv(fix_velocity_csv_path, index=False)
print(f"Saved Fix Velocity data to {fix_velocity_csv_path}")

print("Data extraction complete.")