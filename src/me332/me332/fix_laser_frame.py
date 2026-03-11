#!/usr/bin/env python3
"""
fix_laser_frame.py

修正从Gazebo bridge过来的LaserScan消息的frame_id
将错误的frame_id（如 me332_robot/base_footprint/lidar_2d）修正为 laser_link
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FixLaserFrame(Node):
    def __init__(self):
        super().__init__('fix_laser_frame')
        
        # 订阅原始scan话题
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',  # 从bridge来的原始话题
            self.scan_callback,
            10
        )
        
        # 发布修正后的scan话题
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )
        
        self.get_logger().info('FixLaserFrame node started: /scan_raw -> /scan (frame_id: laser_link)')
    
    def scan_callback(self, msg):
        # 修正frame_id
        msg.header.frame_id = 'laser_link'
        # 发布修正后的消息
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FixLaserFrame()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

