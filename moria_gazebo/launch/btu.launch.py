
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_share=os.path.join(get_package_share_directory('moria_gazebo')) # Directory of this package
    launch_file_dir = os.path.join(get_package_share_directory('moria_gazebo'), 'launch')
    rsp_file_dir = os.path.join(get_package_share_directory('moria_bringup'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(get_package_share_directory('moria_gazebo'),
                        'worlds',
                        'btu.world'
    )

    pkg_share=os.path.join(get_package_share_directory('moria_gazebo')) # Directory of this package

    robot_controllers = os.path.join(pkg_share, 'config', 'robot_controller.yaml')
    # Exporting model path for gazebo 
    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_gazebo_ros, 
                'launch', 
                'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_gazebo_ros, 
                'launch', 
                'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_file_dir, 
                'state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('moria_gazebo'),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory('moria_gazebo'),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )

    spawn_moria_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_file_dir,
                'spawn_moria.launch.py'
            )
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", robot_controllers],
    )


# --------------------------------------------------------------------------




    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_state_cont= Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_controller'],
        )
    
    position_cont=Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller'],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen"
    )

    delayed_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[spawn_moria_cmd],
        )
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_cmd)
    #ld.add_action(joystick)
    ld.add_action(twist_mux)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_moria_cmd)
    ld.add_action(control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(robot_controller_spawner)
    #ld.add_action(delay_joint_state_broadcaster_after_robot_controller_spawner)
    #ld.add_action(controller_manager)
    #ld.add_action(delayed_spawn_entity)
    #ld.add_action(diff_drive_spawner)
    #ld.add_action(joint_broad_spawner)

    return ld