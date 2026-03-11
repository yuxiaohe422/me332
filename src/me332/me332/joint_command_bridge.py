#!/usr/bin/env python3
"""
joint_command_bridge.py
桥接ros2-control的关节命令到Ignition Gazebo
将joint_trajectory_controller的输出转换为Gazebo的关节位置命令
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math


class JointCommandBridge(Node):
    def __init__(self):
        super().__init__('joint_command_bridge')
        
        # 关节名称映射
        self.joint_names = [
            'cone_to_lower',
            'lower_to_middle', 
            'middle_to_upper',
            'gripper_to_finger_left',
            'gripper_to_finger_right'
        ]
        
        # 创建Gazebo关节命令发布者
        self.gazebo_publishers = {}
        for joint_name in self.joint_names:
            # Ignition Gazebo的关节命令话题格式
            topic_name = f'/model/me332_robot/joint/{joint_name}/cmd_pos'
            self.gazebo_publishers[joint_name] = self.create_publisher(
                Float64,
                topic_name,
                10
            )
            self.get_logger().info(f'创建发布者: {topic_name}')
        
        # 订阅joint_states来获取当前关节位置（用于初始化）
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 订阅ros2-control的关节命令（如果有直接命令话题）
        # 注意：joint_trajectory_controller通常通过action接口，但我们也监听joint_states的变化
        
        self.current_positions = {}
        self.get_logger().info('Joint command bridge initialized')
    
    def joint_state_callback(self, msg):
        """从joint_states获取当前关节位置"""
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                if i < len(msg.position):
                    self.current_positions[name] = msg.position[i]
    
    def publish_joint_command(self, joint_name, position):
        """发布关节命令到Gazebo"""
        if joint_name in self.gazebo_publishers:
            msg = Float64()
            msg.data = float(position)
            self.gazebo_publishers[joint_name].publish(msg)
            self.get_logger().debug(f'发布关节命令: {joint_name} = {position}')


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

