Camera-LiDAR Calibration: Step-by-Step Guide
============================================

This repository offers a detailed walkthrough for calibrating a 2D LiDAR with a monocular camera using ROS Kinetic within a Docker environment. It encompasses setup instructions, calibration procedures, and troubleshooting tips to ensure a smooth replication of the calibration process.

Table of Contents
-----------------

*   [Prerequisites](#prerequisites)
    
*   [Docker Setup](#docker-setup)
    
*   [ROS Workspace Setup](#ros-workspace-setup)
    
*   [Building the Calibration Package](#building-the-calibration-package)
    
*   [Running the Calibration Process](#running-the-calibration-process)
    
*   [Collecting Calibration Data](#collecting-calibration-data)
    
*   [Performing Calibration](#performing-calibration)
    
*   [Troubleshooting](#troubleshooting)
    
*   [Acknowledgments](#acknowledgments)
    

Prerequisites
-------------

*   **Operating System**: Ubuntu 16.04 LTS
    
*   **Docker**: Installed and configured
    
*   **Hardware**:
    
    *   Intel RealSense D435i camera
        
    *   YDLIDAR X4PRO LiDAR
        
*   **Software**:
    
    *   ROS Kinetic
        
    *   OpenCV 3.4
        
    *   Ceres Solver
        
    *   PCL (Point Cloud Library)
        

Docker Setup
------------

1.  Pull the Ubuntu 16.04 Image:

``` docker pull ubuntu:16.04 ```
    
2.  Run the Docker Container:

    ```
    docker run -it --name camera_lidar \
      -e DISPLAY=$DISPLAY \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v $HOME:/home/$USER \
        ubuntu:16.04 bash \
    ```

    **Note**: Ensure X11 forwarding is enabled to allow GUI applications like RViz to display.
    
3.  Instal ROS Kinetic
    ```
    apt update && apt install -y curl gnupg2 lsb-release
    echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
    apt update && apt install -y ros-kinetic-desktop-full
    source /opt/ros/kinetic/setup.bash
    ```

    
4.  Install Additional Dependencies
    ```
    apt install -y git vim python-pip tmux
    apt install -y cmake libgoogle-glog-dev libgflags-dev libeigen3-dev libsuitesparse-dev
    ```
    
5.  Install OpenCV 3.4 and Ceres Solver
    **Note**: Ensure that OpenCV and Ceres are properly installed and their paths are correctly set.
    
    *   **OpenCV 3.4**: Follow the official OpenCV installation guide for version 3.4.
        
    *   **Ceres Solver**: Follow the official Ceres Solver installation guide.
        

ROS Workspace Setup
-------------------

1.  Create Catkin Workspace
    ```
    mkdir -p ~/catkin\_ws/src
    cd ~/catkin\_ws/src
    ```
    
2.  Clone the Calibration Repository
    ```
    git clone https://github.com/airou-lab/camera\_lidar\_calibration\_v2.git
    cd ..
    ```
    
3.  Build the workspace
    ```
    catkin\_makesource devel/setup.bash
    ```

    **Note**: If you encounter errors related to missing packages, ensure all dependencies are installed and sourced correctly.
    

Building the Calibration Package
--------------------------------

1.  Navigate to the Package Directory
   ```
    cd ~/catkin_ws/src/camera_lidar_calibration_v2
   ```
2. Verify and Edit CMakeLists.txt (Ensure that all necessary dependencies are included)
```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  dynamic_reconfigure
  tf
  laser_geometry
  message_filters
)

find_package(OpenCV 3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
```
3. Build the Package:
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
**Note:** Address any build errors by ensuring all dependencies are correctly installed and paths are properly set.


Running the Calibration Process
-------------------------------

1.  Start ROS Core
   ```roscore```
    
2.  Launch the Calibration Node and RViz
In a new terminal:
```
roslaunch camera_laser_calibration collect_laser_image_data.launch image_topic:=/camera/color/image_raw

```
**Note:** Ensure that the image_topic parameter matches the topic published by your camera.

3.  Play the ROS Bag File:
In another terminal:
```
rosbag play --pause your_bag_file.bag
```
**Note:** Use the --pause option to control playback and collect data at specific timestamps.


Collecting Calibration Data
---------------------------
1. Set Laser Coordinates:

In a new terminal:
```
rosparam set /collect_laser_image_data/laser_coor "(x, y)"

```
Replace (x, y) with the coordinates obtained from RViz when setting a 2D Nav Goal.

2. Trigger Data Collection:
```
rosparam set /collect_laser_image_data/Save true
```
**Note:** This should open an image window where you can select the laser point.

3. Select the Laser Point:

- Draw a rectangle around the laser point in the image.

- Press the spacebar to confirm the selection.

The selected data will be saved to data/data_v2.txt.

**Note:** Repeat this process for multiple points to improve calibration accuracy.
        

Performing Calibration
----------------------

1.  Launching the Calibration Node
```
roslaunch camera_laser_calibration calibration.launch image_topic:=/camera/color/image_raw
```
**Note:** Ensure that the image_topic parameter matches your camera's topic.

2.  Review Calibration Results:
The calibration results, including the transformation matrix, will be saved to data/data.txt.
    

Troubleshooting
---------------

*   **No Image Window Appears**:
    
    *   Ensure the ROS bag is playing and the image topic is active.
        
    *   Verify that the frame variable in the code is receiving image data.
        
*   **Data Not Saved to data\_v2.txt**:
    
    *   Confirm that the /Save parameter is set to true.
        
    *   Check for any errors in the terminal output.
        
*   **RViz Not Displaying Data**:
    
    *   Verify that the correct fixed frame is set in RViz.
        
    *   Ensure that the LiDAR and camera topics are active.
        
*   **Build Errors**:
    
    *   Check that all dependencies are installed.
        
    *   Ensure that the correct versions of OpenCV and Ceres Solver are used.
        

Acknowledgments
---------------

This calibration tool is based on the work by [xinliangzhong](https://github.com/TurtleZhong/camera_lidar_calibration). Special thanks to the AIROU Lab at the University of Oklahoma for their contributions and support. Please refer to the previous iteration of the repository (https://github.com/airou-lab/camera_lidar_calibration_v2) or the Troubleshooting file in this repository
