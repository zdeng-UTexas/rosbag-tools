# ROS Bag Exact Dataset

This repository contains scripts for extracting image and GPS data from ROS bag files. The primary script, `rosbag_exact_dataset.py`, is a combination of separate extraction scripts to simplify the process of working with image topics and GPS data in ROS bags. 

## Purpose

The script `rosbag_exact_dataset.py`:
- Extracts images from specified topics within a ROS bag file.
- Extracts GPS data from specified topics.
- Interpolates the GPS data to match image timestamps.
- Saves the extracted data in a well-organized output directory structure.

This is particularly useful for generating datasets from ROS bags for machine learning, computer vision, or mapping purposes.

## Usage

To run `rosbag_exact_dataset.py`, ensure that you are in a ROS environment or a Docker container with ROS installed. The script is designed to work with ROS bag files and will fail without the required ROS dependencies.

Example command:
```python rosbag_exact_dataset.py```

## Known Issues

### NumPy and Pandas Compatibility

There is a known issue related to the deprecation of `np.bool` in recent versions of NumPy. If you encounter the following error:
```
AttributeError: module 'numpy' has no attribute 'bool'.
np.bool was a deprecated alias for the builtin bool.
```

You can resolve this issue by upgrading `numpy` and `pandas` to compatible versions:

```
pip install --upgrade numpy pandas
```

For more details, refer to the official NumPy release notes: https://numpy.org/devdocs/release/1.20.0-notes.html#deprecations.

## Files

- **rosbag_exact_dataset.py**: The main script combining image and GPS extraction along with GPS interpolation.
- **rosbag_convert_gps_to_kml.py**: A script to convert GPS data from ROS bag files into KML format.
- **rosbag_exact_gps.py**: A script to extract only GPS data from ROS bag files.
- **rosbag_exact_image.py**: A script to extract only image data from ROS bag files.
- **rosbag_interpolate_gps_for_image.py**: A script to interpolate GPS data to match image timestamps.


Here's an improved version of the markdown formatted for direct use on GitHub:


# Run within ROS Docker

This section outlines the steps to run the necessary scripts within a Docker container with ROS installed.

## Prepare the Container

1. Stop and remove any existing containers to avoid conflicts:

   ```bash
   docker stop ros_container
   docker rm ros_container
   ```

2. Start the ROS container with the required volumes:

   ```bash
   docker run -it --name ros_container \
   -v /robodata/ARL_SARA/GQ-dataset/bagfiles:/rosbags \
   -v /home/zhiyundeng/rosbag-tools:/rosbag-tools \
   -v /robodata/ARL_SARA/GQ-dataset/extracted_data:/extracted_data \
   --user $(id -u):$(id -g) \
   osrf/ros:noetic-desktop-full
   ```

## Set Up ROS Environment

3. In one terminal, create a temporary ROS home directory and start the ROS master:

   ```bash
   mkdir -p /tmp/ros_home
   export ROS_HOME=/tmp/ros_home
   nohup roscore > /tmp/roscore_nohup.log 2>&1 &
   ```

4. In another terminal, access the container:

   ```bash
   # List all running containers (optional)
   docker ps

   # Open a bash shell inside the container as root
   docker exec -it --user root ros_container bash
   ```

5. Source the ROS environment:

   ```bash
   source /opt/ros/noetic/setup.bash
   ```

## Working with ROS Bags

6. Check the information of a ROS bag file:

   ```bash
   rosbag info /rosbags/2024-08-12-11-32-46.bag
   ```

7. Save the rosbag info into a text file:

   ```bash
   ./save_rosbag_info_into_txt.sh
   ```

8. Extract dataset information from ROS bags:

   ```bash
   nohup python3 rosbag_extract_dataset.py &
   ```

## Monitoring

9. Check the status of the running Python script:

   ```bash
   ps aux | grep rosbag_extract_dataset.py
   ```

10. Display the output of the running script:

    ```bash
    tail -f /rosbag-tools/nohup.out
    tail -f /extracted_data/extraction.log
    ```

   
## License

This project is open-source and available under the MIT License.

