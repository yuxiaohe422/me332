#!/usr/bin/env python3
"""
move.launch.py
功能：
- 启动Gazebo仿真环境
- 自动spawn机器人
- 启动避障节点，机器人自动前进并避障
没有要求所以可以认为这个是对雷达的测试。前期这种小东西花了太多时间（？）想着学了还是要做一下，放着吧。
---
修改被注释内容以改变仿真地图/启动rviz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
import xacro


def generate_launch_description() -> LaunchDescription:
    me332_path = get_package_share_directory("me332")
    xacro_file = os.path.join(me332_path, "urdf", "robot_model.xacro")
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否启动RViz'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        # default_value='robot_obstacle_world.sdf',
        default_value='me332_final_project_6_0_ign.sdf',
    )
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        # default_value='robot_obstacle_world',
        default_value='final_map_world',
        description='Gazebo world name inside the SDF (used for bridge topics)',
    )
    use_rviz = LaunchConfiguration('use_rviz')
    world_file = LaunchConfiguration('world')
    world_name = LaunchConfiguration('world_name')

    robot_description = xacro.process_file(xacro_file).toxml()

    world_path = PathJoinSubstitution([me332_path, "world", world_file])
    
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r ", world_path]}.items(),
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": robot_description,
            }
        ],
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_me332_robot",
                output="screen",
                arguments=[
                    "-name", "me332_robot",
                    "-topic", "/robot_description",
                    "-x", "0.0",
                    "-y", "0.1",
                    "-z", "0.1",  # 起始位置稍微抬高一点，避免碰撞
                ],
            )
        ]
    )
    
    bridge = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge_core",
                output="screen",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    PythonExpression([
                        "'/world/' + '", world_name, "' + '/model/me332_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'"
                    ]),
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                    "/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                    "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                    "/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                ],
                remappings=[
                    # 将Gazebo的joint_state话题重映射到ROS标准的/joint_states
                    (
                        PythonExpression([
                            "'/world/' + '", world_name, "' + '/model/me332_robot/joint_state'"
                        ]),
                        "/joint_states",
                    ),
                ],
            )
        ]
    )
    
    rviz_config_path = os.path.join(me332_path, "rviz", "me332_robot.rviz")
    if not os.path.exists(rviz_config_path):
        rviz_config_path = os.path.join(ros_gz_sim_share, "rviz", "robot_description_publisher.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(use_rviz)
    )
    
    obstacle_avoider = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="me332",
                executable="obstacle_avoider",
                name="obstacle_avoider",
                output="screen",
                parameters=[
                    {"use_sim_time": True}
                ],
            )
        ]
    )
    
    return LaunchDescription([
        use_rviz_arg,
        world_arg,
        world_name_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        # rviz,
        obstacle_avoider,  # 自动避障节点
    ])

