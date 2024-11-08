# Wind Turbine Digital Twins

- Dashboard to monitor a wind turbine
- Model for Predictions
- How to use multiple sensors

## Table of Contents
- [Introduction](#introduction)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Introduction
Some introductory text here.

## Installation
Steps to install your project.

Raspberry Libraries Required:
pip install python3-rpi.gpio
pip install Adafruit-GPIO
pip3 install influxdb-client
enable i2c connection
enable serial 
pip3 install adafruit-circuitpython-ina260
pip3 install gpiozero

## Usage
How to use your project.

## Contributing
Guidelines for contributing.

## License
License information.

# Wind Turbine Digital Twin

This project involves creating a digital twin of a wind turbine using ROS Noetic. The digital twin is generated with data from an Xbox Kinect 360 camera, utilizing the `freenect_stack` library to enable LiDAR capabilities for 3D mapping.

## ROS/RViz Dependencies

To use this project, you must install the following dependencies:

- **ROS Noetic**: The ROS distribution used for managing the various packages and nodes required for the digital twin.
- **freenect_stack**: This library enables the Xbox Kinect 360 camera to function with ROS, providing the necessary data to generate a 3D point cloud of the wind turbine.

## Build Instructions

Extract the digital_ws zip 

You are going need to erase previous and do catkin_make

The pathing might be wrong which need to be fixed. 

Make sure to do ~/path/digital_ws/devel/setup.bash

Once you have done all this, you should be able to do

roslaunch freenect_launch freenect.launch 

The command `roslaunch pointcloud_filter filter_kinect.launch` is used to test the program by launching the point cloud filtering process. (This is my code filter)

The program should already open RVIZ for you!

If you want to add a camera, set your topic to /camera/rgb/image_color for color image

Now add PointCloud2 for the Lidar scanner, using `/filtered_points`


![PointCloud2](./images/simulation.png)


## BMEO055
documention for BMEO055
https://cdn-learn.adafruit.com/downloads/pdf/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black.pdf


## MQTT
`mosquitto_sub -h 192.168.1.205 -t wind_turbine/hall_effect`

documention for BMEO055
https://cdn-learn.adafruit.com/downloads/pdf/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black.pdf


