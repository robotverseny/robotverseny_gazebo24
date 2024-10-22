# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('robotverseny_bringup')
    pkg_project_gazebo = get_package_share_directory('robotverseny_gazebo')
    pkg_project_description = get_package_share_directory('robotverseny_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'roboworks', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()


    world_path = os.path.join(pkg_project_gazebo,'worlds/roboworks.sdf')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [' -r -v 1 ' + world_path ], 'on_exit_shutdown': 'True' }.items())
    ## -r means to run the simulation unpaused


    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
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

    # Visualize in RViz with TimerAction to delay starting the node by X seconds (period)
    rviz = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'robotverseny.rviz')],
                condition=IfCondition(LaunchConfiguration('rviz')),
                parameters=[
                    {'use_sim_time': True},
                ]
            )
        ]
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
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

    # Steering and path visualization in RViz with TimerAction to delay starting the node by X seconds (period)
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

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        bridge,
        robot_state_publisher,
        path_and_steer,
        rviz
    ])

