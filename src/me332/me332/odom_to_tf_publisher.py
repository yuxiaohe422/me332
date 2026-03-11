#!/usr/bin/env python3
"""
odom_to_tf_publisher.py - 修正版
从 /odom 消息发布 odom -> base_footprint 的 TF 变换
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_to_tf_publisher')
        
        # 初始化TF广播器（无论是否使用仿真时间）
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 只订阅实际存在的 /odom 主题
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Odom to TF publisher started')
        self.get_logger().info('Subscribing to /odom and publishing odom -> base_footprint TF')
        
        self.log_counter = 0
    
    def odom_callback(self, msg):
        """处理odom消息，发布TF"""
        
        # 创建TF消息
        transform = TransformStamped()
        
        # 使用odom消息的时间戳（这是最重要的！）
        transform.header.stamp = msg.header.stamp
        
        # 设置坐标系
        transform.header.frame_id = 'odom'  # 父坐标系
        transform.child_frame_id = 'base_footprint'  # 子坐标系
        
        # 从odom消息中复制位置和姿态
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        
        transform.transform.rotation.x = msg.pose.pose.orientation.x
        transform.transform.rotation.y = msg.pose.pose.orientation.y
        transform.transform.rotation.z = msg.pose.pose.orientation.z
        transform.transform.rotation.w = msg.pose.pose.orientation.w
        
        # 发布TF
        try:
            self.tf_broadcaster.sendTransform(transform)
            
            # 偶尔打印日志（每50条消息打印一次）
            self.log_counter += 1
            if self.log_counter % 50 == 0:
                self.get_logger().info(
                    f'Published TF: odom -> base_footprint\n'
                    f'  Position: ({transform.transform.translation.x:.3f}, '
                    f'{transform.transform.translation.y:.3f}, '
                    f'{transform.transform.translation.z:.3f})\n'
                    f'  Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}'
                )
                
        except Exception as e:
            self.get_logger().error(f'Failed to publish TF: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OdomToTfPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down odom to TF publisher')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()