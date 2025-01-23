# Moria Description Package
The `moria_description` package is a ROS2 package designed to provide URDF Xacro and mesh files to other packages like `robot_state_publisher`.

## Features
There are two model structure's of Moria. In the urdf directory, the main urdf files are `moria.urdf.xacro` and `moria_fixed.urdf.xacro`.
- `moria.urdf.xacro`: It has movable properties of robot's arm. In this file robot's arm and fingers can move seperately in Rviz and Gazebo.
- `moria_fixed.urdf.xacro`: It has fixed properties of robot's arm. In this file robot's arm and fingers can not move. It has made because arm movements not calculated and Gazebo's performance is very low with free arm.

In this package sensor has defined but all the definitions made for `robot_state_publisher` so sensors are not running in gazebo with this model. Sensor plugins defined in `moria_gazebo` package.

## Package Dependencies
- ament_cmake
- urdf
- rviz2

## Installation
1. Copy this file to workspace
2. Build the workspace
3. Source the workspace

## Usage
This package has not any script. It only serves for `moria_bringup` and `moria_gazebo` packages.