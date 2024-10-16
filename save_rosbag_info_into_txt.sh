#!/bin/bash

# Directory containing the bag files
BAG_DIR="/rosbags"
   
# Output directory where the info will be saved
OUTPUT_DIR="/rosbags/rosbag-info"

# Create the output directory if it doesn't exist
mkdir -p $OUTPUT_DIR

# Loop through all .bag files in the directory
for bagfile in $BAG_DIR/*.bag; do
    # Get the base name of the bag file (without the path)
    base_name=$(basename "$bagfile" .bag)

    # Run rosbag info and save the output to a text file
    rosbag info "$bagfile" > "$OUTPUT_DIR/${base_name}_info.txt"

    echo "Saved info for $bagfile to $OUTPUT_DIR/${base_name}_info.txt"
done
