# object-detector-fusion ROS package
The `object-detector-fusion` is used for detecting and tracking objects from data that is provided by a 2D LiDAR/Laser Scanner and a depth camera.
The detection working principle is largely based on `obstacle_detector` created by Mateusz Przybyla, which used a density-based clustering method to group point clouds and create a geometric representation of objects within the sensor vicinity. The objects are detected by camera first, and then the measurement is corrected by LiDAR detection result, hence the "fusion" name. The dynamic state of the detected objects (position, speed, acceleration) is estimated (tracked) using particle filter. 

This package is designed to work with following sensors:
1. RPLiDAR A1 Laser Scanner
2. Intel RealSense D435 Depth Camera

that are equipped on a land-based autonomous system.

## Requirements
1. [ROS Melodic Morenia](https://wiki.ros.org/melodic)
2. [`rplidar`](https://github.com/robopeak/rplidar_ros)
3. [`realsense-ros`](https://github.com/IntelRealSense/realsense-ros)
4. [`obstacle_detector`](https://github.com/tysik/obstacle_detector)
5. [`jsk_visualization`](https://github.com/jsk-ros-pkg/jsk_visualization)

## Installation
1. Clone and add to your catkin package and run `catkin_make`

## Usage
Run the following commands on the terminal.
To start the fusion nodes:
```
roslaunch object-detector-fusion fusion.launch
```
Start the nodes with only camera:
```
roslaunch object-detector-fusion camera.launch
```
Start the nodes with only lidar (no tracking available):
```
roslaunch object-detector-fusion lidar.launch
```
You should see the output on the rviz main screen.
## Preview
![detect many](https://user-images.githubusercontent.com/36593988/94802503-05e8d700-0412-11eb-90b0-d40df0c20ca5.gif)

![detect one](https://user-images.githubusercontent.com/36593988/94802671-4a747280-0412-11eb-8ce4-dfe56d1e17fb.gif)
