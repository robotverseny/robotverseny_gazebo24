import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Package paths
    pkg_project_bringup = get_package_share_directory('robotverseny_bringup')
    pkg_project_gazebo = get_package_share_directory('robotverseny_gazebo')
    pkg_project_description = get_package_share_directory('robotverseny_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # World path
    world_path = os.path.join(pkg_project_gazebo, 'worlds', 'roboworks.sdf')

    # Launch Gazebo with your world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -v 1 {world_path}',
            'on_exit_shutdown': 'True'
        }.items()
    )

    # RViz (delayed start, optional via launch arg)
    rviz = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'robotverseny.rviz')],
                condition=IfCondition(LaunchConfiguration('rviz')),
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # Bridge ROS ↔ Gazebo topics
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

    # Path + steering visualization (delayed start)
    path_and_steer = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robotverseny_bringup',
                executable='path_and_steering',
                output='screen',
                parameters=[
                    {'publish_steer_marker': True}, 
                    {'marker_topic': 'steer_marker'},
                    {'marker_color': 'g'},
                    {'map_frame': 'odom_combined'},
                    {'marker_frame': 'laser'},
                    {'cmd_topic': 'cmd_vel'},
                    {'use_sim_time': True},
                ]
            )
        ]
    )

    # Static transform publisher (map -> odom_combined)
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom_combined'],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        gz_sim,
        bridge,
        path_and_steer,
        static_map_to_odom,
        rviz
    ])
