#!/usr/bin/env python3
"""
Goal Updater Node for ME332
订阅 /goal_pose 话题，将目标位姿从任意坐标系转换到 map 坐标系，
然后发送给 Nav2 的 navigate_to_pose action
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs


class GoalUpdater(Node):
    def __init__(self):
        super().__init__('goal_updater')
        
        # 创建 TF 缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 订阅 goal_pose 话题
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # 创建 Nav2 action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 创建定时器，定期检查 Nav2 action server 状态
        self.check_timer = self.create_timer(2.0, self.check_nav2_status)
        self.nav2_ready = False
        
        self.get_logger().info('Goal Updater node started')
        self.get_logger().info('Waiting for Nav2 action server...')
        
    def check_nav2_status(self):
        """定期检查 Nav2 action server 状态"""
        if self.nav_to_pose_client.wait_for_server(timeout_sec=0.1):
            if not self.nav2_ready:
                self.nav2_ready = True
                self.get_logger().info('✓ Nav2 action server is ready!')
        else:
            if self.nav2_ready:
                self.nav2_ready = False
                self.get_logger().warn('Nav2 action server is not available')
    
    def goal_callback(self, msg):
        """处理接收到的目标位姿"""
        self.get_logger().info(
            f'Received goal pose: Frame={msg.header.frame_id}, '
            f'Position=({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}), '
            f'Orientation=({msg.pose.orientation.x:.3f}, {msg.pose.orientation.y:.3f}, '
            f'{msg.pose.orientation.z:.3f}, {msg.pose.orientation.w:.3f})'
        )
        
        # 检查 Nav2 是否就绪
        if not self.nav2_ready:
            self.get_logger().warn('Nav2 action server not ready yet, waiting...')
            # 尝试等待更长时间
            if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Nav2 action server not available after waiting!')
                self.get_logger().error('Please check if Nav2 nodes are activated:')
                self.get_logger().error('  ros2 lifecycle list /bt_navigator')
                self.get_logger().error('  ros2 lifecycle list /controller_server')
                return
        
        # 如果已经是 map 坐标系，直接使用
        if msg.header.frame_id == 'map':
            self.send_goal_to_nav2(msg)
            return
        
        # 需要转换坐标系
        try:
            # 等待 TF 变换可用
            transform = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 转换位姿到 map 坐标系
            goal_in_map = self.transform_pose(msg, transform)
            self.get_logger().info(
                f'Transformed goal to map frame: Position=({goal_in_map.pose.position.x:.2f}, '
                f'{goal_in_map.pose.position.y:.2f})'
            )
            
            # 发送给 Nav2
            self.send_goal_to_nav2(goal_in_map)
            
        except TransformException as ex:
            self.get_logger().error(
                f'Failed to transform goal from {msg.header.frame_id} to map: {ex}'
            )
            self.get_logger().warn(
                'Make sure TF tree is properly set up (map -> odom -> base_link)'
            )
    
    def transform_pose(self, pose_stamped, transform):
        """使用 TF 变换转换位姿"""
        # 使用 tf2_geometry_msgs 进行位姿变换
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        pose_transformed.header.frame_id = 'map'
        pose_transformed.header.stamp = self.get_clock().now().to_msg()
        
        return pose_transformed
    
    def send_goal_to_nav2(self, goal_pose):
        """发送目标到 Nav2 action server"""
        # 再次确认 action server 可用
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 action server not available!')
            self.get_logger().error('Please check Nav2 lifecycle nodes:')
            self.get_logger().error('  ros2 lifecycle list /bt_navigator')
            self.get_logger().error('  ros2 lifecycle list /controller_server')
            self.get_logger().error('  ros2 lifecycle list /planner_server')
            self.get_logger().error('  ros2 lifecycle list /amcl')
            return
        
        # 创建 action goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # 发送目标
        self.get_logger().info('Sending goal to Nav2...')
        self.get_logger().info(f'  Position: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        
        # 处理结果
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """处理 action goal 响应"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2!')
            return
        
        self.get_logger().info('Goal accepted by Nav2, navigating...')
        
        # 获取结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """处理导航结果"""
        result = future.result().result
        if result:
            self.get_logger().info('Navigation completed successfully!')
        else:
            self.get_logger().warn('Navigation may have failed')


def main(args=None):
    rclpy.init(args=args)
    node = GoalUpdater()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()





































