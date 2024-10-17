import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('robotverseny_bringup')
    pkg_project_gazebo = get_package_share_directory('robotverseny_gazebo')
    pkg_project_description = get_package_share_directory('robotverseny_description')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    # Load the SDF file from the "description" package
    sdf_file = os.path.join(pkg_project_description, 'models', 'roboworks', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Path to the Gazebo world file
    world_path = os.path.join(pkg_project_gazebo, 'worlds/roboworks.sdf')

    # Include Ignition Gazebo launch file
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={
            'ign_args': f'-r -v 1 {world_path}',  # Run the simulation unpaused
        }.items()
    )

    # Node for publishing robot states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # RViz node to visualize in RViz
    rviz = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'robotverseny.rviz')],
                condition=IfCondition(LaunchConfiguration('rviz')),
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    # Bridge ROS and Ignition Gazebo topics
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': os.path.join(pkg_project_bringup, 'config', 'robotverseny_bridge.yaml')},
            {'use_sim_time': True},
        ],
        output='screen'
    )

    # Node for steering and path visualization
    path_and_steer = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robotverseny_bringup',
                executable='path_and_steering',
                output='screen',
                parameters=[
                    {'publish_steer_marker': True},
                    {'marker_topic': 'roboworks/steer_marker'},
                    {'marker_color': 'g'},
                    {'map_frame': 'map_roboworks'},
                    {'marker_frame': 'lidar_link'},
                    {'cmd_topic': 'roboworks/cmd_vel'},
                    {'use_sim_time': True},
                ]
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        gz_sim,
        bridge,
        robot_state_publisher,
        path_and_steer,
        rviz,
    ])
