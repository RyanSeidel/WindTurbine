# Wind Turbine Digital Twin

This project involves creating a digital twin of a wind turbine using ROS Noetic. The digital twin is generated with data from an Xbox Kinect 360 camera, utilizing the `freenect_stack` library to enable LiDAR capabilities for 3D mapping.

## Dependencies

To use this project, you must install the following dependencies:

- **ROS Noetic**: The ROS distribution used for managing the various packages and nodes required for the digital twin.
- **freenect_stack**: This library enables the Xbox Kinect 360 camera to function with ROS, providing the necessary data to generate a 3D point cloud of the wind turbine.

## Build Instructions

`catkin_make` is used to build the ROS workspace and compile the project.

## Setup Instructions

1. Clone the repository into your ROS workspace.
   ```bash
   cd ~/digital_ws/src
   git clone <repository-url>

    Navigate to your workspace and build the project using catkin_make.

    bash

cd ~/digital_ws
catkin_make

After building, source the setup file to configure your environment.

bash

source ~/digital_ws/devel/setup.bash

Launch the project to visualize the 3D model of the wind turbine using the Kinect LiDAR.

bash

roslaunch <your_launch_file>.launch

## Commands

The command 'roslaunch pointcloud_filter' passthrough_filter.launch is used to test the program by launching the point cloud filtering process.
