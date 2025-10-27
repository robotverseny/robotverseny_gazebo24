# robotverseny Gazebo simulation for `ROS 2`

[![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)
[![Static Badge](https://img.shields.io/badge/ROS_2-Jazzy-34aec5)](https://docs.ros.org/en/jazzy/)
[![Static Badge](https://img.shields.io/badge/Gazebo-Fortress-red)](https://gazebosim.org/docs/fortress/)


![robotverseny_gazebo24_anim](img/sim03.gif)


## Clone 

``` bash
cd ~/ros2_ws/src
```

``` bash
git clone https://github.com/robotverseny/robotverseny_gazebo24
```

## Build

``` bash
cd ~/ros2_ws
```

``` bash
colcon build --symlink-install --packages-select robotverseny_application robotverseny_description robotverseny_bringup robotverseny_gazebo 
```

## Run

<details>
<summary> Don't forget to source before ROS commands.</summary>

``` bash
source ~/ros2_ws/install/setup.bash
```
</details>

``` bash
ros2 launch robotverseny_bringup roboworks.launch.py
```

or without rviz:

``` bash
ros2 launch robotverseny_bringup roboworks.launch.py rviz:=false
```
> [!TIP]
> ## Rviz 2d overlay

>``` r
>sudo apt install ros-humble-rviz-2d-overlay*
>```

> [!WARNING]  
> The command `export IGN_GAZEBO_RESOURCE_PATH=` or `export IGN_GAZEBO_MODEL_PATH=` will delete your previous paths.

## Gazebo related

> [!TIP]
> Gazebo Fortress: [gazebosim.org/docs/fortress/install_ubuntu](https://gazebosim.org/docs/fortress/install_ubuntu), read more about integration: [gazebosim.org/docs/fortress/ros2_integration](https://gazebosim.org/docs/fortress/ros2_integration)
> `ros-gz-bridge` install with a single command: `sudo apt install ros-humble-ros-gz-bridge`


Publish command topic:
``` bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.01}}"
```

Teleop twist keyboard:
``` bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```
    
Ignition info topic:
``` bash
ign topic -i --topic /model/roboworks/cmd_vel
```
Ignition echo topic:

``` bash
ign topic -et /model/roboworks/cmd_vel
```

Topics:

``` bash
ros2 topic list

/clicked_point
/clock
/debug_marker
/debug_marker_left
/debug_marker_left_array
/debug_marker_right
/debug_marker_right_array
/goal_pose
/initialpose
/joint_states
/marker_path
/parameter_events
/robot_description
/roboworks/cmd_vel
/roboworks/odometry
/roboworks/scan
/steer_marker
/steer_marker_array
/rosout
/tf
/tf_static
```

## TF Tree

## Transformations

The frame `/odom_combined` is practically the same as `/map`, there is a static `0,0,0` transform between them. The only dynamic transform is between `/odom_combined` and `/base_link`.

```mermaid

graph TD
    %% Root frame
    map([ map]):::lightd
    odom_combined([ odom_combined]):::light
    base_link([ base_link]):::light
    chassis([ chassis]):::light
    camera_link([ camera_link]):::light
    imu_link([ imu_link]):::light
    laser([ laser]):::light

    %% connections
    odom_combined -.->|dynamic| base_link
    base_link -->|static| chassis
    base_link -->|static| camera_link
    base_link -->|static| imu_link
    base_link -->|static| laser
    map ==>|static - same| odom_combined

classDef light fill:#34aec5,stroke:#152742,stroke-width:2px,color:#152742  
classDef lightd fill:#34aec5,stroke:#152742,stroke-width:2px,color:#152742,stroke-dasharray: 5 5
classDef dark fill:#152742,stroke:#34aec5,stroke-width:2px,color:#34aec5
classDef white fill:#ffffff,stroke:#152742,stroke-width:2px,color:#152742
classDef red fill:#ef4638,stroke:#152742,stroke-width:2px,color:#fff

```

# Tutorials
- [jkk-research.github.io/workshops/f1tenth_sim_a](https://jkk-research.github.io/workshops/f1tenth_sim_a/)
- [sze-info.github.io/ajr/szimulacio/f1tenth_sim_a](https://sze-info.github.io/ajr/szimulacio/f1tenth_sim_a/)
- [Video part 1](https://www.youtube.com/watch?v=90cVRC2Hd7Y)
- [Video part 2](https://www.youtube.com/watch?v=ZlNOnPJfS9c)


# Screenshots
![robotverseny_gazebo24_screenshot01](img/sim04.png)
![robotverseny_gazebo24_screenshot02](img/sim05.png)