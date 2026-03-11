#!/usr/bin/env python3
"""
Voice Control Module using Vosk (支持中文)
基于Vosk离线语音识别控制机器人
本节点可以单独run测试
没下英文语音模型，凑合用中文吧
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import json
import pyaudio
import threading
import math
import time
import os
import cv2
import numpy as np
from vosk import Model, SetLogLevel
from vosk import KaldiRecognizer
VOSK_AVAILABLE = True
VOSK_HAS_KALDI = True

class VoiceTeleop(Node):
    def __init__(self):
        super().__init__('voice_teleop')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Gripper controllers
        self.left_finger_pub = self.create_publisher(
            Float64,
            '/gripper_to_finger_left_position_controller/command',
            10
        )
        self.right_finger_pub = self.create_publisher(
            Float64,
            '/gripper_to_finger_right_position_controller/command',
            10
        )
        
        # Arm controllers
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
        
        if not VOSK_AVAILABLE:
            self.get_logger().error('Vosk not available! Please install: pip install vosk')
            return
        
        # Vosk model path - 优先使用中文模型
        model_path = self.declare_parameter('vosk_model_path', '').value
        if not model_path:
            # 优先尝试中文模型路径（支持多个版本）
            default_paths = [
                # 中文模型（优先）
                os.path.expanduser('~/vosk-model-cn-0.22'),
                os.path.expanduser('~/vosk-model-small-cn-0.22'),
            ]
            for path in default_paths:
                expanded_path = os.path.expanduser(path) if '~' in path else path
                if os.path.exists(expanded_path):
                    model_path = expanded_path
                    self.get_logger().info(f'Found Vosk model at: {model_path}')
                    break
        
        if not model_path or not os.path.exists(model_path):
            self.get_logger().error(
                f'Vosk中文模型未找到！\n'
                f'请从 https://alphacephei.com/vosk/models 下载中文模型\n'
                f'推荐下载: vosk-model-small-cn-0.22 (小模型，速度快)\n'
                f'或: vosk-model-cn-0.22 (完整模型，准确率高)\n'
                f'下载后解压到 ~/vosk-model-cn-0.22 或通过参数 vosk_model_path 指定路径'
            )
            self.get_logger().error('Voice control will not work without Vosk model')
            return
        
        SetLogLevel(-1)  # 减少日志输出
        self.get_logger().info(f'Loading Vosk model from: {model_path}')
        self.model = Model(model_path)
        self.rec = None
        
        # Audio setup
        self.audio_format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.chunk = 4000
        
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=self.audio_format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        
        # State
        self.current_command = None
        self.item_picking_in_progress = False
        self.last_command_time = 0.0
        self.command_cooldown = 2.0  # 命令冷却时间（秒）
        
        # Visualization state
        self.recognized_text = ""  # 最终识别的文本
        self.partial_text = ""  # 部分识别结果
        self.current_action = ""  # 当前执行的动作
        self.last_action_time = 0.0
        
        # Command execution timer
        self.timer = self.create_timer(0.1, self.execute_command)
        
        # Visualization timer (update display window)
        self.viz_timer = self.create_timer(0.05, self._update_display)  # 20 FPS
        
        # Start recognition in separate thread
        self.recognition_thread = threading.Thread(target=self._recognition_loop)
        self.recognition_thread.daemon = True
        self.recognition_thread.start()
        
        self.get_logger().info('Voice Teleop initialized (Vosk)')
        self.get_logger().info('Listening for voice commands...')
        self.get_logger().info('Commands (中文): 前进, 后退, 左转, 右转, 停止, 打开, 关闭, 抓取')
        # self.get_logger().info('Commands (English): forward, backward, turn left, turn right, stop, open, close, pickup')
    
    def _recognition_loop(self):
        """Continuous recognition loop"""
        try:
            # self.rec=self.KaldiRecognizer(self.model, self.rate)
            self.rec = KaldiRecognizer(self.model, self.rate)
            
            self.rec.SetWords(True)
            
            while rclpy.ok():
                try:
                    data = self.stream.read(self.chunk, exception_on_overflow=False)
                    
                    if self.rec.AcceptWaveform(data):
                        result = json.loads(self.rec.Result())
                        if 'text' in result and result['text']:
                            text = result['text'].strip()
                            self.recognized_text = text
                            self.get_logger().info(f'Recognized: {text}')
                            
                            # 检查冷却时间
                            current_time = time.time()
                            if current_time - self.last_command_time > self.command_cooldown:
                                self.current_command = text
                                self.last_command_time = current_time
                    else:
                        # Partial result
                        partial = json.loads(self.rec.PartialResult())
                        if 'partial' in partial and partial['partial']:
                            self.partial_text = partial['partial'].strip()
                            
                except Exception as e:
                    self.get_logger().error(f'Recognition error: {e}')
                    time.sleep(0.1)
                    
        except Exception as e:
            self.get_logger().error(f'Recognition setup error: {e}')
    
    def execute_command(self):
        """Execute the current voice command"""
        if not self.current_command:
            return
        
        command = self.current_command.lower().strip()
        twist = Twist()
        
        if '前进' in command or 'forward' in command or '向前' in command:
            twist.linear.x = 0.4
            self.cmd_vel_pub.publish(twist)
            self.current_action = "FORWARD / 前进"
            self.last_action_time = time.time()
            self.get_logger().info('Command: Forward / 前进')
        elif '后退' in command or 'backward' in command or 'back' in command or '向后' in command:
            twist.linear.x = -0.3
            self.cmd_vel_pub.publish(twist)
            self.current_action = "BACKWARD / 后退"
            self.last_action_time = time.time()
            self.get_logger().info('Command: Backward / 后退')
        elif '左转' in command or 'turn left' in command or 'left' in command:
            twist.angular.z = 1.0
            self.cmd_vel_pub.publish(twist)
            self.current_action = "TURN LEFT / 左转"
            self.last_action_time = time.time()
            self.get_logger().info('Command: Turn Left / 左转')
        elif '右转' in command or 'turn right' in command or 'right' in command:
            twist.angular.z = -1.0
            self.cmd_vel_pub.publish(twist)
            self.current_action = "TURN RIGHT / 右转"
            self.last_action_time = time.time()
            self.get_logger().info('Command: Turn Right / 右转')
        elif '停止' in command or 'stop' in command:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.current_action = "STOP / 停止"
            self.last_action_time = time.time()
            self.get_logger().info('Command: Stop / 停止')
        # NaN.
        elif '打开' in command or 'open' in command:
            finger_msg = Float64()
            finger_msg.data = 0.0
            self.left_finger_pub.publish(finger_msg)
            self.right_finger_pub.publish(finger_msg)
            self.current_action = "OPEN GRIPPER / 打开夹爪"
            self.last_action_time = time.time()
            self.get_logger().info('Command: Open Gripper / 打开夹爪')
        elif '关闭' in command or 'close' in command:
            finger_msg = Float64()
            finger_msg.data = 0.04
            self.left_finger_pub.publish(finger_msg)
            finger_msg.data = -0.04
            self.right_finger_pub.publish(finger_msg)
            self.current_action = "CLOSE GRIPPER / 关闭夹爪"
            self.last_action_time = time.time()
            self.get_logger().info('Command: Close Gripper / 关闭夹爪')
        
        
        # Item picking command
        elif '抓取' in command or 'pickup' in command or 'pick up' in command or '拾取' in command:
            if not self.item_picking_in_progress:
                self.item_picking_in_progress = True
                self.current_action = "PICKUP / 抓取 (进行中...)"
                self.last_action_time = time.time()
                self.get_logger().info('Command: Pickup - Starting item_picking sequence / 开始抓取序列')
                thread = threading.Thread(target=self._item_picking_thread)
                thread.daemon = True
                thread.start()
        
        # Clear command after execution
        self.current_command = None
    
    def _item_picking_thread(self):
        """Execute item_picking sequence in separate thread"""
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
        
        # Return to rest position
        traj_lower = JointTrajectory()
        traj_lower.header.stamp = self.get_clock().now().to_msg()
        traj_lower.joint_names = ['cone_to_lower']
        point_lower = JointTrajectoryPoint()
        point_lower.positions = [0.0]
        point_lower.time_from_start.sec = 2
        traj_lower.points = [point_lower]
        self.arm_lower_pub.publish(traj_lower)
        
        traj_middle = JointTrajectory()
        traj_middle.header.stamp = self.get_clock().now().to_msg()
        traj_middle.joint_names = ['lower_to_middle']
        point_middle = JointTrajectoryPoint()
        point_middle.positions = [0.0]
        point_middle.time_from_start.sec = 2
        traj_middle.points = [point_middle]
        self.arm_middle_pub.publish(traj_middle)
        
        traj_upper = JointTrajectory()
        traj_upper.header.stamp = self.get_clock().now().to_msg()
        traj_upper.joint_names = ['middle_to_upper']
        point_upper = JointTrajectoryPoint()
        point_upper.positions = [0.0]
        point_upper.time_from_start.sec = 2
        traj_upper.points = [point_upper]
        self.arm_upper_pub.publish(traj_upper)
        
        time.sleep(3.0)
        self.item_picking_in_progress = False
        self.current_action = "PICKUP COMPLETED / 抓取完成"
        self.last_action_time = time.time()
        self.get_logger().info('Item picking sequence completed / 抓取序列完成')

    def _update_display(self):
        """
        Update visualization window
        可能是字符编码问题总之这里的小窗口无法显示中文,请自行翻译。(dbq,我懒得改了)
        """
        # Create a black image
        img = np.zeros((400, 800, 3), dtype=np.uint8)
        
        cv2.putText(img, "Voice Control - Voice Teleop", (20, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        if self.partial_text:
            cv2.putText(img, f"Listening: {self.partial_text}", (20, 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 2)
        
        if self.recognized_text:
            cv2.putText(img, f"Recognized: {self.recognized_text}", (20, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if self.current_action:
            elapsed = time.time() - self.last_action_time
            if elapsed < 3.0:
                color = (0, 255, 0)  # Green
                alpha = max(0.3, 1.0 - elapsed / 3.0)
                cv2.putText(img, self.current_action, (20, 200), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
            else:
                self.current_action = ""
    
        status_text = "LISTENING..." if not self.item_picking_in_progress else "PICKING IN PROGRESS..."
        status_color = (0, 255, 0) if not self.item_picking_in_progress else (0, 165, 255)
        cv2.putText(img, status_text, (20, 280), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        help_text = [
            "Commands:",
            "forward, 后退/backward",
            "左转/turn left, 右转/turn right",
            "停止/stop, 打开/open, 关闭/close",
            "抓取/pickup"
        ]
        y_offset = 320
        for i, text in enumerate(help_text):
            color = (200, 200, 200) if i == 0 else (150, 150, 150)
            cv2.putText(img, text, (20, y_offset + i * 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        cv2.imshow('Voice Teleop - Recognition Display', img)
        cv2.waitKey(1)  # Required for OpenCV window to update

def main(args=None):
    rclpy.init(args=args)
    node = VoiceTeleop()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()