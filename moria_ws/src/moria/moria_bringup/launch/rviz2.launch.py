import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node 

import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')
    urdf_name = 'moria.urdf.xacro'

    urdf_file = os.path.join(
        get_package_share_directory('moria_description'),
        'urdf',
        urdf_name
    )

    robot_description_config = Command(['xacro ', urdf_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])


    with open(urdf_file, 'r') as info:
        moria_description = info.read()

    #rsp_params = {'robot_description': moria_description}
    rsp_params = {'robot_description': robot_description_config}

    rviz_config_dir = os.path.join(
        get_package_share_directory('moria_description'),
        'rviz',
        'model.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )
    ])
