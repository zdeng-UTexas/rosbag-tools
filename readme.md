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

## License

This project is open-source and available under the MIT License.
