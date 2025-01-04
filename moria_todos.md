# Moria TODOs
- [/] moria_description 
  - [x] URDF xacro files 
  - [x] Rviz files
  - [ ] add README

- [/] moria_bringup 
  - [x] robot_state_publisher, rviz launch files
  - [ ] add robot.launch.py for run Real Time Robot
  - [ ] add README

- [/] moria_gazebo
  - [x] convert xacro format to urdf format
  - [x] add robot_state_publisher launch file
  - [x] add spawn launch file
  - [x] add empty world and btu like world
  - [x] add empty world launch file
  - [x] add btu world spawn file
  - [x] fix robot piece spawn error
  - [x] Add diff_drive, joint_state_publisher plugins
  - [x] add sensor references
  - [x] fix caster wheel friction problem
  - [x] fix error `waiting /controller_manager/list_controller` ✅ 2024-12-27

- [x] moria_slam ✅ 2024-12-27
	- [[050 moria_simulasyon_kodları#pseudocode]]
  - [x] generate algorithm for following wall ✅ 2024-12-27
  - [x] generate algorithm for turn right when available ✅ 2024-12-27

# Current Errors and Bugs
- [x] There is a dependency error. Add deps to package.xml and CMakeList.txt. 20241119 ✅ 2024-12-27
- [ ] Ros-Iron made packages not works Ros-Foxy. OS depency problem fix it. 20241119
- [x] Check python deps packages 20241119 ✅ 2024-12-27
- [x] I cant open bringup and gazebo ✅ 2024-12-27
 
# Project Ideas and Examples
- turtlebot3_obstacle_detection
  - Detects obstacles and stops
- turtlebot3_position_control
  - Robot moves one point to another with given position and angle
- turtlebot3_nav2
  - basic robot navigation algorithm of ROS
- turtlebot3_node
  - driver node that include diff drive controller, odometry and tf node
- turtlebot3_teleop
  - teleop node for keyboard control