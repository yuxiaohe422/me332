#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class FixOdomFrame(Node):
    def __init__(self):
        super().__init__('fix_odom_frame')
        self.use_sim_time = self.get_parameter_or('use_sim_time', False)
        # subscribe to possible sources
        self.sub1 = self.create_subscription(Odometry, '/model/me332_robot/odometry', self.cb, 10)
        self.sub2 = self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.get_logger().info('fix_odom_frame started, subscribing to /model/me332_robot/odometry and /odom')

    def cb(self, msg: Odometry):
        # ignore messages that already use correct frame id
        if msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_link':
            return
        out = Odometry()
        out.header = msg.header
        # set correct frame ids
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'
        out.child_frame_id = msg.child_frame_id if msg.child_frame_id else 'base_link'
        out.pose = msg.pose
        out.twist = msg.twist
        # publish corrected message
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FixOdomFrame()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


