import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import time
from cv_bridge import CvBridge
import threading
import gradio as gr
import threading
import matplotlib.pyplot as plt
from PIL import Image as PILImage

from robot_interface.util_interface import RobotArmInterface
from robot_interface.ur3_robot_arm_control import UR3RobotArmControl
from robot_interface.robotiq85_control import Robotiq85
from utils.aruco_detector import ArucoDetector



# 全局变量来存储最新的数据
latest_image = None
latest_data = None
control_status = "未启动"
# 存储每个维度的历史数据
history_data = [[] for _ in range(7)]
# 线程锁
image_lock = threading.Lock()
data_lock = threading.Lock()

class Ur3DeltaPoseController(Node):
    def __init__(self,
                 arm_control: RobotArmInterface = None,
                 tracker_topic='/trackerA/pose',
                 img_pub_topic='/device_image',
                 pos_pub_topic='/device_pose_data'):
        super().__init__('ur3_delta_pose_controller')
        # Get parameters
        self.declare_parameter('delta_pose_topic', tracker_topic)
        self.delta_pose_topic = self.get_parameter('delta_pose_topic').get_parameter_value().string_value

        # Initialize the Robot Arm Control
        if arm_control is None:
            self.arm_control = UR3RobotArmControl()  # 默认使用 UR3
        else:
            self.arm_control = arm_control

        self.arm_control.connect()

        # Target pose variables
        actual_q = self.arm_control.get_current_pose()
        self.get_logger().info(f"Position obtained from the robot: {actual_q}")
        self.target_pose = actual_q
        self.init_current_pose = False
        self.current_pose = self.target_pose

        # Subscribe to the /delta_pose topic
        self.delta_pose_sub = self.create_subscription(
            PoseStamped,
            self.delta_pose_topic,
            self.delta_pose_callback,
            10  # QoS depth
        )
        self.get_logger().info("Ur3DeltaPoseController node has started, waiting for /delta_pose messages...")

        try:
            self.aruco_detector = ArucoDetector()
            self.gripper = Robotiq85()
            self.get_logger().info("ArucoDetector and Robotiq85 initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Initialization error: {e}")
            raise e
        self.gripper.open()
        self.gripper_state = "open"

        self.bridge = CvBridge()
        self.stop_threads = False
        self.paused = False
        self.vrpose_data = [0.0] * 7

        # 启动线程
        self.gripper_thread = threading.Thread(target=self.gripper_control, daemon=True)
        self.image_thread = threading.Thread(target=self.publish_image, daemon=True)
        self.data_thread = threading.Thread(target=self.publish_data, daemon=True)

        self.gripper_thread.start()
        self.image_thread.start()
        self.data_thread.start()

        self.get_logger().info("All threads started.")

    def gripper_control(self):
        self.get_logger().info("Gripper control thread started.")
        while not self.stop_threads:
            try:
                if not self.paused:
                    gripper_distance = self.aruco_detector.get_aruco_pixel_distance()
                    self.get_logger().info(f"gripper distance: {gripper_distance}")
                    if self.gripper and gripper_distance > 250 and self.gripper_state == "close":
                        self.gripper.open()
                        self.gripper_state = "open"
                        self.get_logger().info("Gripper opened.")
                    elif self.gripper and gripper_distance < 250 and self.gripper_state == "open":
                        self.gripper.close()
                        self.gripper_state = "close"
                        self.get_logger().info("Gripper closed.")
                time.sleep(0.05)
            except Exception as e:
                self.get_logger().error(f"Exception in gripper_control thread: {e}")
                break

    def publish_image(self):
        global latest_image
        while not self.stop_threads:
            if not self.paused:
                cv_image = self.aruco_detector.get_aruco_img()
                if cv_image is not None:
                    with image_lock:
                        latest_image = cv_image
            time.sleep(0.05)  # 20 Hz

    def publish_data(self):
        global latest_data
        while not self.stop_threads:
            if not self.paused:
                self.vrpose_data[6] = self.aruco_detector.get_aruco_pixel_distance()
                with data_lock:
                    latest_data = self.vrpose_data.copy()
                    for i in range(7):
                        history_data[i].append(latest_data[i])
                        if len(history_data[i]) > 100:  # 只保留最近100个数据点
                            history_data[i].pop(0)
            time.sleep(0.1)  # 20 Hz

    def delta_pose_callback(self, delta_pose_msg):
        if self.paused:
            return
        # Parse the position and orientation information from the message
        self.vrpose_data = [
            delta_pose_msg.pose.position.x,
            delta_pose_msg.pose.position.y,
            delta_pose_msg.pose.position.z,
            delta_pose_msg.pose.orientation.x,
            delta_pose_msg.pose.orientation.y,
            delta_pose_msg.pose.orientation.z,
            self.vrpose_data[-1]
        ]
        x, y, z, roll, pitch, yaw, _ = self.vrpose_data
        # Convert coordinates
        next_x = x
        next_y = -y
        next_z = z
        next_ry = -roll
        next_rz = pitch
        next_rx = yaw
        # Calculate the increments
        delta_x = next_x - self.current_pose[0]
        delta_y = next_y - self.current_pose[1]
        delta_z = next_z - self.current_pose[2]
        delta_rx = next_rx - self.current_pose[3]
        delta_ry = next_ry - self.current_pose[4]
        delta_rz = next_rz - self.current_pose[5]
        self.current_pose = [next_x, next_y, next_z, next_rx, next_ry, next_rz]
        if not self.init_current_pose:
            self.init_current_pose = True
            return
        self.get_logger().info(
            f"Received delta pose: Δx={delta_x:.4f}, Δy={delta_y:.4f}, Δz={delta_z:.4f}, \nΔrx={delta_rx:.4f}, Δry={delta_ry:.4f}, Δrz={delta_rz:.4f}"
        )
        # Update the target pose
        self.target_pose[0] += delta_x
        self.target_pose[1] += delta_y
        self.target_pose[2] += delta_z
        self.target_pose[3] += delta_rx
        self.target_pose[4] += delta_ry
        self.target_pose[5] += delta_rz
        target = list(self.target_pose)  # Copy the target pose
        delta_x_list = [delta_x, delta_y, delta_z, delta_rx, delta_ry, delta_rz]
        delta_is_error = self.check_extreme_values_fixed_threshold(delta_x_list, 0.0, 0.1)
        if delta_is_error:
            self.get_logger().info(f"Target pose is error: {delta_x_list}")
        # Send the position command
        speed = 1.0
        acceleration = 1.0
        servo_time = 1.0 / 30
        lookahead_time = 0.03
        gain = 1000
        if not delta_is_error:
            # arm move control
            t_start = self.arm_control.rtde_c.initPeriod()
            self.arm_control.move_to_pose(target, speed, acceleration, servo_time, lookahead_time, gain)
            self.arm_control.rtde_c.waitPeriod(t_start)

    def check_extreme_values_fixed_threshold(self, lst, min_threshold, max_threshold):
        for num in lst:
            abs_num = abs(num)
            if abs_num < min_threshold or abs_num > max_threshold:
                return True
        return False

    def shutdown(self):
        self.get_logger().info("Shutting down Ur3DeltaPoseController node.")
        self.stop_threads = True
        # 等待线程结束
        self.gripper_thread.join(timeout=1)
        self.image_thread.join(timeout=1)
        self.data_thread.join(timeout=1)
        self.get_logger().info("All threads have been stopped.")
        self.aruco_detector.stop_detect()
        self.arm_control.disconnect()

    def pause(self):
        self.paused = True
        self.get_logger().info("Control paused.")

    def resume(self):
        self.paused = False
        self.get_logger().info("Control resumed.")


controller = None
ros_spin_thread = None

def start_ur3_control():
    global controller, ros_spin_thread
    if controller is not None:
        return "控制已启动。"

    # 初始化 ROS2
    rclpy.init()

    # 创建 Ur3DeltaPoseController 实例
    controller = Ur3DeltaPoseController()

    # 启动 rclpy.spin 在后台线程
    def ros_spin():
        try:
            rclpy.spin(controller)
        except Exception as e:
            print(f"ROS Spin Error: {e}")

    ros_spin_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_spin_thread.start()

    return "UR3 控制已启动。"

def pause_ur3_control():
    if controller:
        controller.pause()
        return "控制已暂停。"
    return "控制器未初始化。"

def resume_ur3_control():
    if controller:
        controller.resume()
        return "控制已恢复。"
    return "控制器未初始化。"

def get_latest_image():
    global latest_image
    with image_lock:
        if latest_image is not None:
            return latest_image
    return np.zeros((480, 640, 3), dtype=np.uint8)  # 返回空白图像作为占位

def get_latest_data_image():
    global history_data
    fig, axes = plt.subplots(3, 1, figsize=(6, 6))
    for i in range(3):
        axes[i].plot(history_data[i])
        axes[i].set_title(f"Dimension {i+1}")
    plt.tight_layout()
    fig.canvas.draw()
    image = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    plt.close(fig)
    return PILImage.fromarray(image)


if __name__ == '__main__':
    with gr.Blocks() as demo:
        gr.Markdown("## LightURDI——遥操作")
        with gr.Row():
            start_button = gr.Button("系统启动", scale=1)
            # shutdown_button = gr.Button("系统停止", scale=1)
            pause_button = gr.Button("暂停控制", scale=1)
            resume_button = gr.Button("恢复控制", scale=1)
            output_text = gr.Textbox(label="状态信息", interactive=False, scale=4)

        with gr.Row():
            image_output = gr.Image(label="Aruco 图像", scale=4)
            data_output = gr.Image(label="设备数据曲线", scale=3)

        # 设置按钮的点击事件
        start_button.click(fn=start_ur3_control, outputs=output_text)
        pause_button.click(fn=pause_ur3_control, outputs=output_text)
        resume_button.click(fn=resume_ur3_control, outputs=output_text)
        # shutdown_button.click(fn=shutdown_ur3_control, outputs=output_text)

        demo.load(fn=get_latest_image, inputs=None, outputs=image_output, every=0.1)
        demo.load(fn=get_latest_data_image, inputs=None, outputs=data_output, every=0.1)

    demo.launch()
