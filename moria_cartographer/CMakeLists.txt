cmake_minimum_required(VERSION 3.5)
project(moria_cartographer)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(nav2_bringup REQUIRED)

install(
    DIRECTORY config launch run rviz
    DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
    run/wall_track.py
    run/frontier_exploration.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
