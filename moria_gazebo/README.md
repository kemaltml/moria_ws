# Moria Robot Package

This package contains the necessary resources and configurations for simulating and operating the Moria robot in ROS2. The package includes launch files, configuration files, models, and world files tailored for both simulation and real-world deployment.

---

## Package Contents

### **1. Root Files**
- **`CMakeLists.txt`**: Configuration for building the ROS2 package.
- **`package.xml`**: Package metadata and dependencies.
- **`README.md`**: Documentation for the package.

---

### **2. Configuration Files (`config/`)**
Contains YAML configuration files for various components:
- **`gazebo_params.yaml`**: Parameters for Gazebo simulation.
- **`gaz_ros2_ctl_use_sim.yaml`**: Controller configuration for simulation.
- **`joystick.yaml`**: Configuration for joystick control.
- **`my_controllers.yaml`**: Custom controller parameters.
- **`robot_controller.yaml`**: Robot-specific controller settings.
- **`twist_mux.yaml`**: Twist multiplexer configuration.

---

### **3. Launch Files (`launch/`)**
Includes Python launch files for different use cases:
- **`btu_big.launch.py`**: Launches the larger BTU model simulation.
- **`btu.launch.py`**: Launches the BTU simulation.
- **`empty_world.launch.py`**: Launches an empty simulation world.
- **`joystick.launch.py`**: Enables joystick control.
- **`spawn_moria.launch.py`**: Spawns the Moria robot in a simulation.
- **`state_publisher.launch.py`**: Publishes robot state data.

---

### **4. Models (`models/`)**
Contains 3D models and configuration files for the robot and environment:
- **`btu_real/`**: Contains the larger BTU model assets and SDF configuration.
  - **`meshes/`**: 3D assets for the BTU model.
  - **`model.config`**: Metadata for the BTU model.
  - **`model.sdf`**: Simulation Description Format (SDF) file for BTU.

- **`moria/`**: Contains assets for the Moria robot.
  - **`meshes/`**: 3D parts of the Moria robot.
  - **`model.config`**: Metadata for the Moria robot.
  - **`model.sdf`**: Simulation Description Format file for Gazebo.
  - **`model.urdf`**: Unified Robot Description Format file for ROS2.

---

### **5. RViz Configurations (`rviz/`)**
- **`model.rviz`**: RViz configuration for visualizing the robot model.

---

### **6. World Files (`worlds/`)**
Simulation environments for Gazebo:
- **`btu_real.world`**: World with the BTU real model.
- **`btu.world`**: Standard BTU simulation world.
- **`empty_world.world`**: A minimal, empty world for testing.

---

## Features
- **Simulation**: Launch files and world files for simulating the robot in Gazebo.
- **Control**: Configurations for joystick and custom controllers.
- **Visualization**: Preconfigured RViz setup for robot state and model visualization.
- **Extensibility**: Modular structure for adding new models, configurations, or simulation environments.

---

## Requirements
- ROS2 Humble or later
- Gazebo (compatible with your ROS2 version)
- geometry_msgs
- nav_msgs
- rclcpp
- rclpy
- sensor_msgs
- tf2
- moria_description

---

## Usage
1. Copy the package into ROS2 workspace
2. Build workspace
3. Source workspace
4. Launch the desired script