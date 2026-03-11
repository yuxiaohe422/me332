#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  #(check this)
import cv2
import mediapipe as mp
import numpy as np
import math
import json
import time

# 绝世容颜跳脸预警

class HandGestureTeleop(Node):
    def __init__(self):
        super().__init__('hand_gesture_teleop')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.left_finger_pub  = self.create_publisher(Float64, '/gripper_to_finger_left_position_controller/command', 10)
        self.right_finger_pub = self.create_publisher(Float64, '/gripper_to_finger_right_position_controller/command', 10)

        # Arm controllers for item_picking
        self.arm_lower_pub = self.create_publisher(
            JointTrajectory,
            '/arm_lower_controller/joint_trajectory',
            10
        )
        self.arm_middle_pub = self.create_publisher(
            JointTrajectory,
            '/arm_middle_controller/joint_trajectory',
            10
        )
        self.arm_upper_pub = self.create_publisher(
            JointTrajectory,
            '/arm_upper_controller/joint_trajectory',
            10
        )

        # ---------- MediaPipe ----------
        # 使用与官方demo类似的参数设置，提高检测准确率
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
            # 这两个参数用这不舒服可以调
        )
        self.mp_draw = mp.solutions.drawing_utils

        # ---------- State variables ----------
        self.last_gripper_state = "open"
        self.ok_gesture_detected = False
        self.ok_gesture_start_time = 0.0
        self.item_picking_in_progress = False
        
        # 手势平滑处理（减少抖动）
        self.gesture_history = []
        self.history_size = 5
        self.last_stable_gesture = None

        # Camera setup
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera. Please check camera connection.')
            raise RuntimeError('Camera not available')
        
        # 设置摄像头分辨率（如果支持）
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        self.get_logger().info('Camera initialized successfully')

        self.timer = self.create_timer(0.05, self.loop)  # 20Hz
        self.get_logger().info('Hand Gesture Teleop initialized')

        # Debug session identifiers for backward gesture issue
        self._debug_session_id = "debug-session"
        self._debug_run_id = "backward-gesture-1"

    def loop(self):
        success, img = self.cap.read()
        if not success:
            return

        img = cv2.flip(img, 1)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)

        twist = Twist()
        gripper_cmd = None   # None表示保持当前状态，0.0=打开，0.04=关闭

        if results.multi_hand_landmarks:
            for hand_lms in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(img, hand_lms, self.mp_hands.HAND_CONNECTIONS)

                # Get keypoints
                lm = hand_lms.landmark
                wrist = lm[0]
                index_tip = lm[8]
                middle_tip = lm[12]
                ring_tip = lm[16]
                pinky_tip = lm[20]
                thumb_tip = lm[4]

                # 获取关键点（重新定义，确保使用正确的landmark）
                thumb_tip = lm[4]
                thumb_ip = lm[3]
                thumb_mcp = lm[2]
                index_tip = lm[8]
                index_pip = lm[6]
                index_mcp = lm[5]
                middle_tip = lm[12]
                middle_pip = lm[10]
                middle_mcp = lm[9]
                ring_tip = lm[16]
                ring_pip = lm[14]
                ring_mcp = lm[13]
                pinky_tip = lm[20]
                pinky_pip = lm[18]
                pinky_mcp = lm[17]
                wrist = lm[0]

                # 计算手掌方向
                palm_vec = np.array([middle_tip.x - wrist.x, middle_tip.y - wrist.y])
                palm_angle = math.atan2(palm_vec[1], palm_vec[0])

                # 改进的手指检测逻辑，参考MediaPipe官方demo实现
                fingers_up = []
                
                # 拇指：判断是否向右伸出（x坐标，因为拇指是横向的）
                fingers_up.append(thumb_tip.x > thumb_ip.x)
                
                # 其他四指：判断是否向上伸出（y坐标，值越小越靠上）
                # 使用更简单的判断：tip.y < pip.y < mcp.y
                for tip, pip, mcp in [
                    (index_tip, index_pip, index_mcp),
                    (middle_tip, middle_pip, middle_mcp),
                    (ring_tip, ring_pip, ring_mcp),
                    (pinky_tip, pinky_pip, pinky_mcp)
                ]:
                    fingers_up.append(tip.y < pip.y < mcp.y)

                up_count = sum(fingers_up)
                
                # 手势平滑处理
                current_gesture = {
                    'up_count': up_count,
                    'fingers': fingers_up.copy(),
                    'palm_angle': palm_angle
                }
                self.gesture_history.append(current_gesture)
                if len(self.gesture_history) > self.history_size:
                    self.gesture_history.pop(0)
                
                # 使用历史中最常见的手势（减少抖动）
                if len(self.gesture_history) >= 3:
                    recent_counts = [g['up_count'] for g in self.gesture_history[-3:]]
                    most_common_up_count = max(set(recent_counts), key=recent_counts.count)
                    if most_common_up_count != up_count:
                        # 如果当前手势与历史不一致，使用历史值
                        up_count = most_common_up_count

                # ---------- Gesture Recognition ----------
                # 1. Forward (index finger up)
                if up_count == 1 and fingers_up[1]:
                    twist.linear.x = 0.4
                    self._agent_log(
                        "H1",
                        "Forward gesture detected",
                        {"twist_linear_x": twist.linear.x, "up_count": int(up_count)}
                    )
                    cv2.putText(img, "FORWARD", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3)

                # 2. Backward (victory gesture - index and middle)
                elif up_count == 2 and fingers_up[1] and fingers_up[2]:
                    twist.linear.x = -0.3
                    self._agent_log(
                        "H1",
                        "Backward gesture detected",
                        {"twist_linear_x": twist.linear.x, "up_count": int(up_count)}
                    )
                    cv2.putText(img, "BACKWARD", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 3)

                # 3. Turn left / right (open palm)
                elif up_count >= 4:
                    if -2.5 < palm_angle < -0.8:   # Palm left
                        twist.angular.z = 1.0
                        cv2.putText(img, "TURN LEFT", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 3)
                    elif 0.8 < palm_angle < 2.5:   # Palm right
                        twist.angular.z = -1.0
                        cv2.putText(img, "TURN RIGHT", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 3)

                # 4. Thumb up/down detection (优先检测，避免与其他手势冲突)
                thumb_up = False
                thumb_down = False
                # 拇指向上：拇指tip的y坐标明显小于thumb_ip（向上伸出）
                if thumb_tip.y < thumb_ip.y - 0.05:
                    thumb_up = True
                    cv2.putText(img, "THUMB UP", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 3)
                # 拇指向下：拇指tip的y坐标明显大于thumb_ip（向下伸出）
                elif thumb_tip.y > thumb_ip.y + 0.05:
                    thumb_down = True
                    cv2.putText(img, "THUMB DOWN", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 3)
                
                # 5. Fist detection (所有手指都弯曲，包括拇指)
                # 握拳：除了拇指可能伸出外，其他四指都应该弯曲
                is_fist = (not fingers_up[1] and not fingers_up[2] and 
                          not fingers_up[3] and not fingers_up[4])
                # 如果拇指也弯曲，则更确定是握拳
                if is_fist and not thumb_up:
                    gripper_cmd = 0.04  # Close
                    if self.last_gripper_state != "closed":
                        self.get_logger().info("Gripper: Close (Fist)")
                        self.last_gripper_state = "closed"
                    cv2.putText(img, "FIST", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255), 3)
                elif not is_fist:
                    # 不是握拳时，检查是否需要打开夹爪（全掌张开）
                    if up_count == 5:  # All fingers up (including thumb)
                        gripper_cmd = 0.0   # Open
                        if self.last_gripper_state != "open":
                            self.get_logger().info("Gripper: Open")
                            self.last_gripper_state = "open"
                
                # 6. OK gesture → item_picking (改进检测，避免与握拳冲突)
                # OK手势：拇指和食指形成圆圈，其他手指基本伸直或稍微弯曲
                thumb_index_dist = math.hypot(thumb_tip.x - index_tip.x, thumb_tip.y - index_tip.y)
                # 更严格的判断：
                # 1. 拇指和食指距离很近（形成圆圈）
                # 2. 中指、无名指、小指应该基本伸直（up_count应该>=2，因为有拇指和食指）
                # 3. 不是握拳状态
                middle_ring_pinky_up = fingers_up[2] or fingers_up[3] or fingers_up[4]
                is_ok_gesture = (thumb_index_dist < 0.03 and  # 拇指食指很接近
                               up_count >= 2 and  # 至少拇指和食指向上
                               not is_fist and  # 不是握拳
                               middle_ring_pinky_up)  # 其他手指至少有一个向上
                
                if is_ok_gesture:
                    # 需要持续检测一段时间才触发（避免误触发）
                    current_time = self.get_clock().now().seconds_nanoseconds()[0]
                    if not self.ok_gesture_detected:
                        self.ok_gesture_start_time = current_time
                        self.ok_gesture_detected = True
                    elif current_time - self.ok_gesture_start_time > 1.0:  # 持续1秒才触发
                        if not self.item_picking_in_progress:
                            self.item_picking_in_progress = True
                            self.get_logger().info('OK gesture detected (stable), starting item_picking')
                            self.execute_item_picking()
                            self.ok_gesture_detected = False  # 重置
                        cv2.putText(img, "OK -> PICKING!", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,165,255), 4)
                    else:
                        cv2.putText(img, f"OK ({int(current_time - self.ok_gesture_start_time)}s)", 
                                  (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,165,255), 2)
                else:
                    self.ok_gesture_detected = False
                
                # 注意：夹爪控制已在握拳检测和全掌张开检测中处理
                # 如果既不是握拳也不是全掌张开，保持当前状态

        else:
            # No hand detected → stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish base velocity
        self._agent_log(
            "H2",
            "Publishing cmd_vel from hand_gesture",
            {
                "twist_linear_x": twist.linear.x,
                "twist_angular_z": twist.angular.z,
            },
        )
        self.cmd_vel_pub.publish(twist)

        # Publish gripper position (only if command is set)
        if gripper_cmd is not None:
            finger_msg = Float64()
            finger_msg.data = gripper_cmd
            self.left_finger_pub.publish(finger_msg)
            finger_msg.data = -gripper_cmd  # Right finger (reflect=-1)
            self.right_finger_pub.publish(finger_msg)

        cv2.imshow("Hand Gesture Teleop", img)
        if cv2.waitKey(1) == 27:  # ESC to exit
            rclpy.shutdown()

    def execute_item_picking(self):
        """Execute item_picking action sequence"""
        self.get_logger().info('Executing item_picking sequence...')
        
        # Import item_picking module functions
        import threading
        thread = threading.Thread(target=self._item_picking_thread)
        thread.daemon = True
        thread.start()
    
    def _item_picking_thread(self):
        """Item picking in separate thread"""
        import time
        
        # Move arm to picking position
        CONE_TO_LOWER_ANGLE = math.radians(38.76)
        LOWER_TO_MIDDLE_ANGLE = math.radians(26.64)
        MIDDLE_TO_UPPER_ANGLE = math.radians(63.36)
        
        # Lower joint
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ['cone_to_lower']
        point = JointTrajectoryPoint()
        point.positions = [CONE_TO_LOWER_ANGLE]
        point.time_from_start.sec = 2
        traj.points = [point]
        self.arm_lower_pub.publish(traj)
        
        # Middle joint
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ['lower_to_middle']
        point = JointTrajectoryPoint()
        point.positions = [LOWER_TO_MIDDLE_ANGLE]
        point.time_from_start.sec = 2
        traj.points = [point]
        self.arm_middle_pub.publish(traj)
        
        # Upper joint
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ['middle_to_upper']
        point = JointTrajectoryPoint()
        point.positions = [MIDDLE_TO_UPPER_ANGLE]
        point.time_from_start.sec = 2
        traj.points = [point]
        self.arm_upper_pub.publish(traj)
        
        time.sleep(3.0)
        
        # Open gripper
        finger_msg = Float64()
        finger_msg.data = 0.0
        self.left_finger_pub.publish(finger_msg)
        self.right_finger_pub.publish(finger_msg)
        time.sleep(1.0)
        
        # Close gripper
        finger_msg.data = 0.04
        self.left_finger_pub.publish(finger_msg)
        finger_msg.data = -0.04
        self.right_finger_pub.publish(finger_msg)
        time.sleep(2.0)
        
        # Return to rest
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ['cone_to_lower', 'lower_to_middle', 'middle_to_upper']
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0]
        point.time_from_start.sec = 2
        traj.points = [point]
        self.arm_lower_pub.publish(traj)
        self.arm_middle_pub.publish(traj)
        self.arm_upper_pub.publish(traj)
        
        time.sleep(3.0)
        self.item_picking_in_progress = False
        self.get_logger().info('Item picking sequence completed')

def main(args=None):
    rclpy.init(args=args)
    node = HandGestureTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
