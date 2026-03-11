#!/usr/bin/env python3
"""
Obstacle Avoider Module
Based on lidar sensor, avoids obstacles when distance < OBSTACLE_THRESHOLD + fixed angular velocity
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math  # For inf and nan checks

MAX_RANGE = 3.0  # Maximum detection range in meters
MIN_RANGE = 0.1  # Minimum reliable range to ignore self-detection/noise
OBSTACLE_THRESHOLD = 1.5  # Increased threshold for safer avoidance
TURN_ANGULAR_VELOCITY = 0.6  # Fixed angular velocity for turning
FORWARD_VELOCITY = 0.3  # Forward velocity when no obstacle
FRONT_HALF_FOV_RAD = math.radians(30)  # Consider +/-30 degrees as “front”

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Keep the last command and publish it periodically to avoid deadman at startup
        self.last_cmd = Twist()
        self.last_cmd.linear.x = FORWARD_VELOCITY
        self.last_cmd.angular.z = 0.0
        self.timer = self.create_timer(0.1, self.publish_last_cmd)  # 10 Hz
        
        # Subscriber for laser scan
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.get_logger().info('Obstacle Avoider initialized')
        self.get_logger().info(f'MAX_RANGE: {MAX_RANGE}m, MIN_RANGE: {MIN_RANGE}m, Threshold: {OBSTACLE_THRESHOLD}m')
    
    def laser_callback(self, msg):
        """Process laser scan data and avoid obstacles"""
        if not msg.ranges:
            return
        
        # If angle metadata is missing, fall back to index-based processing
        use_angles = abs(msg.angle_increment) > 1e-6

        valid_samples = []
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and MIN_RANGE < r < MAX_RANGE:
                if use_angles:
                    angle = msg.angle_min + i * msg.angle_increment
                else:
                    angle = 0.0  # treat as front if we don't know
                valid_samples.append((angle, r))

        if not valid_samples:
            # No valid readings, continue forward
            self.publish_twist(FORWARD_VELOCITY, 0.0)
            return

        # Front sector: +/- FRONT_HALF_FOV_RAD
        front_ranges = [r for angle, r in valid_samples if abs(angle) <= FRONT_HALF_FOV_RAD]
        if not front_ranges:
            # No valid front readings, assume clear and go forward
            self.publish_twist(FORWARD_VELOCITY, 0.0)
            return

        min_front_distance = min(front_ranges)

        command = Twist()

        if min_front_distance < OBSTACLE_THRESHOLD:
            # Obstacle detected, decide turn direction by free space
            left_min = min((r for angle, r in valid_samples if angle > 0.0), default=MAX_RANGE)
            right_min = min((r for angle, r in valid_samples if angle < 0.0), default=MAX_RANGE)

            if left_min > right_min:
                command.angular.z = TURN_ANGULAR_VELOCITY
                self.get_logger().debug(f'Obstacle at {min_front_distance:.2f}m, turning left')
            else:
                command.angular.z = -TURN_ANGULAR_VELOCITY
                self.get_logger().debug(f'Obstacle at {min_front_distance:.2f}m, turning right')

            command.linear.x = 0.0  # Stop forward motion while turning for better avoidance
        else:
            # No obstacle, move forward
            command.linear.x = FORWARD_VELOCITY
            command.angular.z = 0.0

        self.last_cmd = command
        self.cmd_vel_pub.publish(command)
    
    def publish_twist(self, linear_x, angular_z):
        """Helper method to publish twist command"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.last_cmd = twist
        self.cmd_vel_pub.publish(twist)

    def publish_last_cmd(self):
        """Periodic republish to keep robot moving even before scan arrives."""
        self.cmd_vel_pub.publish(self.last_cmd)

def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    try:
        rclpy.spin(avoider)
    except KeyboardInterrupt:
        pass
    finally:
        avoider.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()