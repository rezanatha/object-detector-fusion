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
4. [`depthimage-to-laserscan`](https://github.com/ros-perception/depthimage_to_laserscan)
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
To obtain 2D depth data from 3D depth data, this package use `depthimage-to-laserscan` which subscribes `sensor_msgs/Image` type message in `camera/depth/image_rect_raw` and publishes `sensor_msgs/LaserScan` message, the 2D depth data that has been converted to laserscan-type message. `camera_obstacle_extractor` node converts this message into set of objects which are publishes as custom type `obstacles_detector/Obstacles`. `lidar_obstacle_extractor` does the same to the 
`sensor_msgs/LaserScan` published by `rplidarNode`. Green circle with red concentric circle represents detected objects from camera data. Beige and blue circle represents detected objects from lidar data.

`particle_filter` node will use the measurement data from these detected objects results to estimate dynamic state of an object as well as reduce measurement noise. A magenta box represents mean of the particles used to approximate the state and black dots surrounding the box represents the aforementioned particles.

![detect many](https://user-images.githubusercontent.com/36593988/94839251-eddc7c00-0440-11eb-9d85-dc3f94e5c033.gif)

![detect one](https://user-images.githubusercontent.com/36593988/94837496-8291aa80-043e-11eb-8cca-c75239333d82.gif))
