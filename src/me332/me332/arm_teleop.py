#!/usr/bin/env python3
"""
Simple keyboard teleop for arm joints.
Run with:
  python3 /home/yxh/ros2_ws/src/me332/scripts/arm_teleop.py
Controls (increment/decrement by step):
  q/a : base_to_cone +/-
  w/s : cone_to_lower +/-
  e/d : lower_to_middle +/-
  r/f : middle_to_upper +/-
  t/g : gripper_to_finger_left +/-
  y/h : gripper_to_finger_right +/-
  x   : send current positions (explicit)
  Ctrl-C to exit
"""
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = [
    "base_to_cone",
    "cone_to_lower",
    "lower_to_middle",
    "middle_to_upper",
    "gripper_to_finger_left",
    "gripper_to_finger_right",
]

KEY_MAP = {
    "q": ("base_to_cone", +0.1),
    "a": ("base_to_cone", -0.1),
    "w": ("cone_to_lower", +0.1),
    "s": ("cone_to_lower", -0.1),
    "e": ("lower_to_middle", +0.1),
    "d": ("lower_to_middle", -0.1),
    "r": ("middle_to_upper", +0.1),
    "f": ("middle_to_upper", -0.1),
    "t": ("gripper_to_finger_left", +0.02),
    "g": ("gripper_to_finger_left", -0.02),
    "y": ("gripper_to_finger_right", +0.02),
    "h": ("gripper_to_finger_right", -0.02),
}


def getch(timeout=0.1):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([fd], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
        else:
            ch = None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


class ArmTeleop(Node):
    def __init__(self):
        super().__init__("arm_teleop")
        self._action_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.current_positions = {name: 0.0 for name in JOINT_NAMES}
        # try to wait for server a short while
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("FollowJointTrajectory action server not available yet.")

    def send_goal(self, positions_dict, duration_sec=1.0):
        # build trajectory message with joints present in positions_dict
        joint_names = list(positions_dict.keys())
        positions = [positions_dict[name] for name in joint_names]

        traj = JointTrajectory()
        traj.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        traj.points = [point]

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        if send_goal_future.done():
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().warning("Goal rejected by action server")
                return False
            # don't wait for result here; fire-and-forget
            self.get_logger().info(f"Goal accepted for joints: {joint_names}")
            return True
        else:
            self.get_logger().warning("send_goal future did not complete")
            return False


def print_help():
    print(__doc__)


def main(argv=None):
    rclpy.init(args=argv)
    node = ArmTeleop()
    try:
        print_help()
        print("Current positions:", node.current_positions)
        while rclpy.ok():
            ch = getch(timeout=0.1)
            if ch is None:
                # allow spinning so action client futures progress
                rclpy.spin_once(node, timeout_sec=0.01)
                continue
            if ch == "\x03":  # Ctrl-C
                break
            if ch == "x":
                # send whole current_positions
                node.send_goal(node.current_positions)
                continue
            key = ch.lower()
            if key in KEY_MAP:
                joint, delta = KEY_MAP[key]
                node.current_positions[joint] = node.current_positions.get(joint, 0.0) + delta
                print(f"{joint} -> {node.current_positions[joint]:.3f}")
                # send only the changed joint as partial goal (controller allows partial)
                node.send_goal({joint: node.current_positions[joint]}, duration_sec=0.8)
            else:
                # ignore other keys
                pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


