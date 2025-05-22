## Camera-Lidar Calibration Tool (ROS Version)

### Overview

This repository provides tools for calibrating a 2D LiDAR with a monocular camera. The current pipeline supports ROS Kinetic and has been tested using the Intel RealSense D435i camera and YDLIDAR X4PRO. Calibration leverages dynamic reconfigure and Ceres Solver for optimization.

---

### Prerequisites

* Ubuntu 16.04 (tested)
* ROS Kinetic
* OpenCV 3.4+
* Ceres Solver
* Docker (optional but recommended for reproducibility)

---

### Updated Features

* Persistent Docker container support
* Debugged and verified dynamic reconfigure functionality
* Helper scripts for manual laser coordinate setting
* Compatible with Intel RealSense D435i data format

---

### Directory Structure

```
catkin_ws/
|├── src/
|   └── camera_lidar_calibration_v2/
|       |
|       ├── config/            # config.yaml with fx, fy, cx, cy, k1, k2, p1, p2
|       ├── data/              # data.txt and data_v2.txt outputs
|       ├── launch/            # collect_laser_image_data.launch, calibration.launch
|       ├── rviz/              # show.rviz configuration
|       ├── src/               # C++ files
|       └── cfg/               # dynamic_range.cfg for sliders
```

---

### Installation and Setup

```bash
# Clone the repo into your catkin workspace
cd ~/catkin_ws/src
git clone https://github.com/airou-lab/camera_lidar_calibration_v2.git
cd ..

# Build the workspace
catkin_make
source devel/setup.bash
```

---

### Running the System

1. Start roscore in a terminal.
2. Launch the main calibration interface:

```bash
roslaunch camera_laser_calibration collect_laser_image_data.launch image_topic:=/camera/color/image_raw
```

3. In another terminal, play your bag file:

```bash
rosbag play --pause your_data.bag
```

* Make sure your docker container saves your progress
* Make sure you have downloaded the ROS bag to your downloads to be able to play it later
* If you need anything installed apt-get for Debian commands is your friend (nano, vim, git, etc.)
* Learn how to use tmux
* If you end making significant changes in your catkin_ws don't forget to remake the catkin_ws and source devel/setup.bash
* Make sure your instrinsic values are correct for your devices 
* Your order of running should 1- Roscore 2- Rosbag 3- Roslaunch
* Make sure your opencv and ceres solver is up to date and properly running

4. Click 2D Nav Goal in RViz to select laser points.

---

### NEW: Helper Script

To automatically set `/laser_coor` and trigger `/Save true`, use:

```bash
./set_laser_coor.sh
```

This script prompts you for (x, y) coordinates and sets them in ROS before triggering the save process.

---

### Troubleshooting

**Q: RQT dynamic reconfigure is blank?**

* Check that the `collect_laser_image_data` node is running with the correct name.
* Make sure you've `catkin_make`'d after editing the `.cfg` file.
* Confirm that the dynamic reconfigure `.cfg` file is correctly included in `CMakeLists.txt` with `generate_dynamic_reconfigure_options(...)` and `add_dependencies(...)`.

**Q: Setting `/Save true` does nothing?**

* Your bag must be playing and `frame` (the camera image) must be populated. If no image is received, the GUI window won’t pop up.
* Check whether the camera topic is publishing with `rostopic echo /camera/color/image_raw`.
* You may need to wait a few seconds after starting playback before setting `/Save`.

**Q: No `/parameter_updates` topics?**

* Launch file or node name may be broken. Confirm that your node is properly registered in the launch file and the executable name matches.
* Check that the node name in `main()` matches the one expected by dynamic reconfigure.

**Q: RViz opens but no LiDAR scan appears?**

* Verify that the `/scan` topic is being published and appears in `rostopic list`.
* Make sure RViz is subscribed to `/scan` under the correct frame (e.g., `laser_frame`).

**Q: collect\_laser\_image\_data crashes with OpenCV errors?**

* Ensure that OpenCV 3.4 is installed in your Docker image and linked properly.
* You may need to manually install or symlink `libopencv_core.so.3.4`.

**Q: `data_v2.txt` is still empty after clicking and saving?**

* You may not have drawn a rectangle on the popped-up image window. This step is essential for corner detection.
* Try increasing the contrast or quality of the image for better corner detection.

---

### Calibration Output

Once enough laser-image correspondences are collected (usually 15-20), run:

```bash
roslaunch camera_laser_calibration calibration.launch image_topic:=/camera/color/image_raw
```

Output will be saved in `data/data.txt` with the final transformation matrix from LiDAR to camera.

---

### Future Improvements

* Port support for ROS Noetic and ROS2
* Integrate RViz plugins for smoother UI
* Provide more robust error messages for data collection edge cases
* Automate rectangle selection using image processing

---

### Maintainers & Contributors

* Aksel Sozudogru (2025 Spring Contributor)

  * Debugged core node setup, added helper script, improved README
  * Docker-based build testing
  * Identified bugs with OpenCV linkage and parameter propagation
