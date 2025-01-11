cmake_minimum_required(VERSION 3.5)
project(moria_gazebo)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
    add_compile_definitions(_USE_MATH_DEFINES)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(moria_description REQUIRED)

pkg_check_modules(GAZEBO REQUIRED gazebo)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}    
)

set(dependencies 
    "geometry_msgs"
    "nav_msgs"
    "rclcpp"
    "rclpy"
    "sensor_msgs"
    "tf2"
    "moria_description"
)

install(DIRECTORY config launch models rviz worlds
    DESTINATION share/${PROJECT_NAME}
)



ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(moria_description)
ament_package()