# csi_camera

This package provides functionality for handling CSI cameras in ROS2.

## Features

- Automatic detection of connected CSI cameras
- Launch file for running camera nodes
- Publishing camera images as ROS2 topics

## Usage

To run the camera nodes:

```
ros2 launch my_camera_pkg auto_cameras.launch.py
currently optimized to the ROS2 FOXY and
Humble should be also considered
```

## Dependencies

- ROS2
- OpenCV (4.5.0)
- cv_bridge

## License

TODO: Add license information