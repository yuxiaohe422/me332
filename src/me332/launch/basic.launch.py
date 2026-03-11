#!/usr/bin/env python3
"""
basic.launch.py
使用 Ignition Gazebo (ros_gz_sim) 启动基本仿真环境
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro
from launch.actions import GroupAction


def get_world_path(context):
    """获取世界文件路径，支持 .world 和 .sdf 文件"""
    me332_path = get_package_share_directory("me332")
    world_file_str = context.perform_substitution(LaunchConfiguration('world', default='room.sdf'))
    # world_file_str = context.perform_substitution(LaunchConfiguration('world',default='me332_final_project_6_0_ign.sdf'))
    world_path = os.path.join(me332_path, "world", world_file_str)

    if world_file_str.endswith('.sdf'):
        world_file_base = world_file_str[:-4]  # 移除 .sdf 扩展名
        world_file_alt = world_file_base + '.world'
        world_path_world = os.path.join(me332_path, "world", world_file_alt)
        
        # 优先使用 .world 文件（如果存在）
        if os.path.exists(world_path_world):
            print(f"[INFO] Using .world file: {world_file_alt} (instead of {world_file_str})")
            return world_path_world
        elif os.path.exists(world_path):
            print(f"[INFO] Using .sdf file: {world_file_str}")
            return world_path
        else:
            print(f"[WARN] File not found: {world_file_str}, trying .world alternative")
            return world_path_world  # 即使不存在也返回，让 Gazebo 报错
    elif world_file_str.endswith('.world'):
        if os.path.exists(world_path):
            print(f"[INFO] Using .world file: {world_file_str}")
            return world_path
        else:
            print(f"[ERROR] World file not found: {world_path}")
            return world_path  # 让 Gazebo 报错
    else:
        # 尝试 .world 和 .sdf
        world_path_world = world_path + '.world'
        world_path_sdf = world_path + '.sdf'
        if os.path.exists(world_path_world):
            print(f"[INFO] Using .world file: {world_file_str}.world")
            return world_path_world
        elif os.path.exists(world_path_sdf):
            print(f"[INFO] Using .sdf file: {world_file_str}.sdf")
            return world_path_sdf
        else:
            print(f"[WARN] Neither .world nor .sdf file found for: {world_file_str}")
            return world_path_world  # 默认尝试 .world


def launch_setup(context, *args, **kwargs):
    """Launch setup function using OpaqueFunction"""
    me332_path = get_package_share_directory("me332")
    xacro_file = os.path.join(me332_path, "urdf", "robot_model.xacro")
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_path = get_world_path(context)
    robot_description_xml = xacro.process_file(xacro_file).toxml()
    # Replace ROS1-style $(find pkg) occurrences with absolute package path
    # so xacro-generated URDF doesn't contain unresolved $(find ...) tokens.
    robot_description_xml = robot_description_xml.replace("$(find me332)", me332_path)
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    # gz_args needs to be a string with space between -r and path
    gz_args_str = f"-r {world_path}"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args_str}.items(),
    )

    rviz_config_path = os.path.join(me332_path, "rviz", "mapping.rviz")
    if not os.path.exists(rviz_config_path):
        rviz_config_path = os.path.join(me332_path, "rviz", "urdf_config.rviz")
        if not os.path.exists(rviz_config_path):
            rviz_config_path = os.path.join(ros_gz_sim_share, "rviz", "robot_description_publisher.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description_xml,
            }
        ],
    )
    
    # 发布 robot_description 到话题（spawn 节点需要从话题订阅）
    robot_description_publisher = Node(
        package="me332",
        executable="robot_description_publisher",
        name="robot_description_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description_xml,
            }
        ],
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_me332_robot",
                output="screen",
                arguments=[
                    "-name", "me332_robot",
                    "-topic", "robot_description",  # 话题名
                    "-x", "0.0",
                    "-y", "0.0",
                    "-z", "0.1",
                ],
                parameters=[
                    {"use_sim_time": use_sim_time},
                ],
            )
        ]
    )

    controllers_yaml = os.path.join(me332_path, "config", "ros2_controller.yaml")

    # 不需要手动启动 ros2_control_node
    controller_manager = TimerAction(
        period=3.5,  # 在 spawn_robot 后启动
        actions=[
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                name="controller_manager",
                output="screen",
                parameters=[
                    {"robot_description": robot_description_xml},
                    controllers_yaml,
                    {"use_sim_time": use_sim_time},
                ],
            )
        ]
    )

    slam_params_file = os.path.join(get_package_share_directory("me332"), "config", "slam_toolbox_params.yaml")

    slam_tool = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )
    
    bridge = GroupAction(
        actions=[
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package="ros_gz_bridge",
                        executable="parameter_bridge",
                        name="ros_gz_bridge_core",
                        output="screen",
                        arguments=[
                            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                            "/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                        ],
                    )
                ]
            )
        ]
    )

    load_controllers = TimerAction(
        period=6.0,  # 在 spawn_robot 后，等待插件完全初始化
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager"
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "arm_controller",
                    "--controller-manager", "/controller_manager"
                ],
                output="screen",
            ),
        ]
    )
    
    odom_to_tf = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='me332',
                executable='odom_to_tf_publisher',
                name='odom_to_tf_publisher',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time}
                ],
            )
        ]
    )
    
    return [
        gazebo,
        rviz,
        robot_state_publisher,
        robot_description_publisher,
        # controller_manager,  # gz_ros2_control 插件会自动创建
        load_controllers,
        bridge,
        slam_tool,
        spawn_robot,
        odom_to_tf,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value='room.sdf',
            # default_value='me332_final_project_6_0_ign.sdf',
            description='World file name under the me332/world folder. Supports both .world and .sdf files. If .sdf is specified, .world file will be used if available.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
