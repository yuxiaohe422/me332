#!/usr/bin/env python3
"""
robot_description_publisher.py
用于将robot_description参数发布到话题
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__('robot_description_publisher')
        
        # 从参数服务器获取robot_description
        self.declare_parameter('robot_description', '')
        robot_description = self.get_parameter('robot_description').get_parameter_value().string_value
        
        if not robot_description:
            self.get_logger().error('robot_description parameter is empty!')
            return
        
        # 创建发布者 - 发布到 robot_description 话题（不带斜杠，与 spawn 节点匹配）
        self.publisher_ = self.create_publisher(String, 'robot_description', 1)
        
        # 创建定时器，定期发布robot_description（每秒一次）
        self.timer = self.create_timer(1.0, self.publish_description)
        
        self.robot_description = robot_description
        self.get_logger().info('Robot description publisher started')
        self.get_logger().info(f'Publishing robot_description (length: {len(robot_description)} chars)')
    
    def publish_description(self):
        msg = String()
        msg.data = self.robot_description
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RobotDescriptionPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import sys
        print(f"Error in robot_description_publisher: {e}", file=sys.stderr)
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            # Ignore shutdown errors (e.g., if already shut down by launch system)
            pass


if __name__ == '__main__':
    main()