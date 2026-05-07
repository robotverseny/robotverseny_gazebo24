import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, AppendEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    pkg_project_bringup = get_package_share_directory('robotverseny_bringup')
    pkg_project_gazebo = get_package_share_directory('robotverseny_gazebo')
    pkg_project_description = get_package_share_directory('robotverseny_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 1. Útvonal a modellekhez (a model:// hivatkozások feloldásához)
    models_path = os.path.join(pkg_project_description, 'models')
    
    # 2. Útvonal a share gyökérhez (a package:// hivatkozások feloldásához a Gazebo számára)
    share_parent_path = os.path.dirname(pkg_project_description)
    
    # A két útvonal összekapcsolása kettősponttal (Linux szabvány)
    resource_paths = f"{models_path}:{share_parent_path}"

    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_paths
    )

    sdf_file = os.path.join(pkg_project_description, 'models', 'roboworks', 'model.sdf')
    
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    world_path = os.path.join(pkg_project_gazebo, 'worlds', 'roboworks.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -v 1 {world_path}',
            'on_exit_shutdown': 'True'
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
            {'frame_prefix': ''}
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {
                'config_file': os.path.join(pkg_project_bringup, 'config', 'robotverseny_bridge.yaml'),
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            },
            {'use_sim_time': True},
        ],
        output='screen'
    )

    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom_combined'],
        parameters=[{'use_sim_time': True}]
    )
    
    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom_combined', 'base_link'],
        parameters=[
            {'use_sim_time': True} 
        ]
    )

    base_to_roboworks = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_roboworks',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'roboworks'],
        parameters=[{'use_sim_time': True}]
    )

    base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0',
                   'base_link', 'roboworks/lidar_link/gpu_lidar'],
        parameters=[{'use_sim_time': True}]
    )

    lidar_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0',
                   'roboworks/lidar_link/gpu_lidar', 'laser'],
        parameters=[{'use_sim_time': True}]
    )

    path_and_steer = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robotverseny_bringup',
                executable='path_and_steering',
                output='screen',
                parameters=[
                    {'publish_steer_marker': True},
                    {'map_frame': 'odom_combined'},
                    {'marker_frame': 'laser'},
                    {'use_sim_time': True},
                ]
            )
        ]
    )

    rviz = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d',
                           os.path.join(pkg_project_bringup, 'config', 'robotverseny.rviz')],
                condition=IfCondition(LaunchConfiguration('rviz')),
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        set_gz_resource_path,
        gz_sim,
        bridge,
        static_map_to_odom,
        odom_to_base,
        base_to_roboworks,
        robot_state_publisher,
        base_to_lidar,
        lidar_to_laser,
        path_and_steer,
        rviz
    ])