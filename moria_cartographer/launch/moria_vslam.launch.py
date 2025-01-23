
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'camera_point_frame',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True,
          'use_action_for_goal':True,
          'Reg/Force3DoF':'true',
          'Grid/RayTracing':'true', # Fill empty space
          'Grid/3D':'false', # Use 2D occupancy
          'Grid/RangeMax':'3.0',
          'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
          'Grid/MaxGroundHeight':'0.05', # All points above 5 cm are obstacles
          'Grid/MaxObstacleHeight':'0.3',  # All points over 1 meter are ignored
          'Grid/FromOccupancyGrid': 'True',         # Use external 2D map for 3D map updates
          'Grid/MapFrameProjection': 'True',       # Project the 2D map into the 3D map
          'RGBD/UpdateOccupancyGrid': 'True',      # Dynamically update the 3D map
          'Grid/2DMapTopic': '/map',
          'RGBD/UpdateOccupancyGrid': 'True',
          'DetectionRate': '0.5',
          'RGBD/StartFrame': '5',
          'Mem/IncrementalMemory':'True',
          'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
          'Optimizer/Robust': 'false',
          'RGBD/OptimizeMaxError': '1.5',
          'RGBD/UseOdomFeatures': 'false',  # Don’t refine odometry with features
          'RGBD/ProximityByTime': 'false',  # Disable proximity detection for scans
          'Reg/Strategy': '0',
          'ICP/CorrespondenceRatio': '0.01', # Reduce minimum correspondence ratio
          'ICP/MaxTranslation': '0.3',       # Increase allowed translation (meters)
          'ICP/MaxRotation': '0.2',          # Increase allowed rotation (radians)
          'RGBD/LinearUpdate': '0.05',   # Reduce linear distance threshold (default 0.2)
          'RGBD/AngularUpdate': '0.05'
    }

    remappings=[
          ('rgb/image', '/camera_rgb/image_raw'),
          ('rgb/camera_info', '/camera_rgb/camera_info'),
          ('depth/image', '/camera_depth/depth/image_raw'),
          ('/map', '/map2')]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        # Nodes to launch

        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)

        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters],
            remappings=remappings),

        # Obstacle detection with the camera for nav2 local costmap.
        # First, we need to convert depth image to a point cloud.
        # Second, we segment the floor from the obstacles.
        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[{'decimation': 2,
                         'max_depth': 3.0,
                         'voxel_size': 0.02}],
            remappings=[('depth/image', '/camera/depth/image_raw'),
                        ('depth/camera_info', '/camera_depth/depth/camera_info'),
                        ('cloud', '/camera_depth/points')]),
        Node(
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[parameters],
            remappings=[('cloud', '/camera_depth/points'),
                        ('obstacles', '/camera/obstacles'),
                        ('ground', '/camera/ground')]),
    ])
