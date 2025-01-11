
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    This function generates a launch description for a Gazebo simulation environment.
    It sets up various components such as Gazebo server, Gazebo client, robot state publisher,
    joystick, twist mux, spawner for the robot, and controller manager.

    Parameters:
    None

    Returns:
    LaunchDescription: A launch description object containing all the necessary actions for the simulation.
    """
    pkg_share=os.path.join(get_package_share_directory('moria_gazebo')) # Directory of this package
    print(pkg_share)
    launch_file_dir = os.path.join(get_package_share_directory('moria_gazebo'), 'launch')
    print(launch_file_dir) 
    rsp_file_dir = os.path.join(get_package_share_directory('moria_bringup'), 'launch') # We run Robot State Publisher from moria_description because why not??
    print(rsp_file_dir)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    print(pkg_gazebo_ros)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(get_package_share_directory('moria_gazebo'),
                        'worlds',
                        'btu_real.world'
    )
    print(world)

    # Exporting model path for gazebo 
    gazebo_models_path = os.path.join(pkg_share, 'models')
    print(gazebo_models_path)
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
                rsp_file_dir,
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

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
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
    #ld.add_action(controller_manager)
    #ld.add_action(delayed_spawn_entity)
    #ld.add_action(diff_drive_spawner)
    #ld.add_action(joint_broad_spawner)

    return ld