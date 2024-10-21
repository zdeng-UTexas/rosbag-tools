import rosbag
import csv
import rospy
from nav_msgs.msg import Odometry

# Function to extract odometry data
def extract_odometry_data(bag_file, output_csv):
    # Open the bag file
    bag = rosbag.Bag(bag_file, 'r')

    # Open the CSV file for writing the extracted data
    with open(output_csv, mode='w') as csv_file:
        csv_writer = csv.writer(csv_file)
        # Write the CSV header
        csv_writer.writerow(['timestamp', 'position_x', 'position_y', 'position_z', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w'])

        # Iterate through each message in the topic /odom (or your odometry topic)
        for topic, msg, t in bag.read_messages(topics=['/trevor/odom']):
            # Extract the timestamp, position, and orientation
            timestamp = t.to_sec()
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation

            # Write the data to the CSV file
            csv_writer.writerow([timestamp, position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w])

    print(f"Odometry data extracted to {output_csv}")

    # Close the bag file
    bag.close()

# Main function
if __name__ == '__main__':
    # Path to your bag file
    # bag_file = '/home/zd3534/phoenix-r1/bags/2024-08-14-14-12-48.bag'
    bag_file = '/home/zd3534/phoenix-r1/bags/2024-08-14-14-25-35.bag'
    # Path to output CSV file
    # output_csv = '/home/zd3534/phoenix-r1/bags/odometry_2024-08-14-14-12-48.csv'
    output_csv = '/home/zd3534/phoenix-r1/bags/odometry_2024-08-14-14-25-35.csv'

    # Call the extraction function
    extract_odometry_data(bag_file, output_csv)
