import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model_folder = 'models/moria'
    urdf_file_name = 'model.urdf'
    urdf_file_path = os.path.join(get_package_share_directory('moria_gazebo'),
                                    model_folder, urdf_file_name)
    
    pkg_share=os.path.join(get_package_share_directory('moria_gazebo'))  
    
    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    declare_x_position_cmd = DeclareLaunchArgument(
                            'x_pose', default_value='0.0',
                            description='Specify namespace of the robot')
    
    declare_y_position_cmd = DeclareLaunchArgument(
                            'y_pose', default_value='0.0',
                            description='Specify namespace of the robot')
    
    start_gazebo_ros_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'Moria',
            '-file', urdf_file_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.1'
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(start_gazebo_ros_spawner)

    return ld