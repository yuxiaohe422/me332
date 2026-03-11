from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

"""
advanced.launch.py
高级启动文件，整合基本仿真、手势控制、语音控制。
不加启动参数运行launch文件，手势识别和语音识别的窗口都会出现，但是语音识别功能用不了，手势识别的优先级更高。
"""

def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )
    ld.add_action(use_sim_time_arg)

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="room.sdf",
        description="World SDF filename under the me332/world folder",
    )
    ld.add_action(world_arg)

    me332_share = FindPackageShare("me332")
    basic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([me332_share, "launch", "basic.launch.py"])
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    ld.add_action(basic_launch)

    use_hand_gesture_arg = DeclareLaunchArgument(
        'use_hand_gesture',
        default_value='true',
        description='Enable hand gesture teleoperation'
    )
    ld.add_action(use_hand_gesture_arg)
    
    hand_gesture_teleop = Node(
        package="me332",
        executable="hand_gesture_teleop",
        name="hand_gesture_teleop",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("use_hand_gesture")),
    )
    ld.add_action(hand_gesture_teleop)

    use_voice_teleop_arg = DeclareLaunchArgument(
        'use_voice_teleop',
        default_value='true',
        description='Enable voice teleoperation (requires Vosk Chinese model)'
    )
    ld.add_action(use_voice_teleop_arg)
    
    vosk_model_path_arg = DeclareLaunchArgument(
        'vosk_model_path',
        default_value='',
        description='Path to Vosk Chinese model (default: ~/vosk-model-cn-0.22)'
    )
    ld.add_action(vosk_model_path_arg)
    
    voice_teleop = Node(
        package="me332",
        executable="voice_teleop",
        name="voice_teleop",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "vosk_model_path": LaunchConfiguration("vosk_model_path"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("use_voice_teleop")),
    )
    ld.add_action(voice_teleop)

    return ld