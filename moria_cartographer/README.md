# Moria Robot Package

This package contains the necessary scripts and configurations for mapping with the Moria robot in ROS2.

---

## Package Contents

### **1. Root Files**
- **`CMakeLists.txt`**: Configuration for building the ROS2 package.
- **`package.xml`**: Package metadata and dependencies.
- **`README.md`**: Documentation for the package.

---

### **2. Configuration Files (`config/`)**
- **`moria_slam.yaml`**: Parameters for slam_toolbox mapping.

---

### **3. Launch Files (`launch/`)**
Includes Python launch files for different use cases:
- **`moria_cartograph.launch.py`**: Launches the slam_toolbox, rviz and wall track algorithm.
- **`moria_vslam_scan.launch.py`**: This launch file configures RTAB-Map SLAM or localization with RGB-D input and obstacle detection.
- **`moria_vslam.launch.py`**:This launch file sets up RTAB-Map SLAM or localization with enhanced obstacle detection and occupancy grid updates for navigation.

---

### **4. Script Files (`run/`)**
Includes Python script files for navigation process of robot.
- **`frontier_exploration.py`**: This moves the robot to unknown locations after wall track mapping.
- **`map_clear.py`**: This script filters a 3D point cloud based on a 2D occupancy map, removing points in free space while preserving colors and applying noise reduction.
- **`trim.py`**: This script detects the windows and fixes the map.
- **`wall_track.py`**: This script finds a wall and tracks it with basic PD controller algorithm.

---

### **5. RViz Configurations (`rviz/`)**
- **`moria_slam.rviz`**: RViz configuration for visualizing the robot model.

---

## Features
- Movement control of robot
- 2D and 3D mapping
- Wall tracking
- Exploring and fixing the unknown places
- Glass detection
- Map correction

---

## Requirements
- ROS2 Humble or later
- Gazebo (compatible with your ROS2 version)
- slam_toolbox
- nav_bringup
- rclcpp
- rclpy
- sensor_msgs
- tf2
- moria_gazebo
- opencv2

---

## Usage
1. Copy the package into ROS2 workspace
2. Build workspace
3. Source workspace
4. Spawn the robot
5. Start mapping process
6. Start wall track script
7. After wall track start frontier exploration
