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

# from tmp_files.robotiq85_realsense2 import Robotiq85Gripper
from robot_interface.util_interface import RobotArmInterface
from robot_interface.ur3_robot_arm_control import UR3RobotArmControl 
from robot_interface.robotiq85_control import Robotiq85
from utils.aruco_detector import ArucoDetector


class Ur3DeltaPoseController(Node):
    def __init__(self, 
                 arm_control: RobotArmInterface = None,
                 tracker_topic='/trackerA/pose', 
                 img_pub_topic='/device_image', 
                 pos_pub_topic='/device_pose_data'):
        # Initialize the ROS2 node
        super().__init__('ur3_delta_pose_controller')
        # Get parameters
        self.declare_parameter('delta_pose_topic', tracker_topic)
        self.delta_pose_topic = self.get_parameter('delta_pose_topic').get_parameter_value().string_value

        # Initialize the Robot Arm Control
        if arm_control is None:
            self.arm_control = UR3RobotArmControl()  # 默认使用UR3
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
        # Gripper threading
        
        # Image and data publisher
        self.image_publisher = self.create_publisher(Image, img_pub_topic, 10)
        self.data_publisher = self.create_publisher(Float32MultiArray, pos_pub_topic, 10)
        self.bridge = CvBridge()
        self.stop_threads = False
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
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"Exception in gripper_control thread: {e}")
                break

    def publish_image(self):
        while not self.stop_threads:
            cv_image = self.aruco_detector.get_aruco_img()
            if cv_image is not None:
                image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                self.image_publisher.publish(image_msg)
            time.sleep(0.1)  # 10 Hz

    def publish_data(self):
        while not self.stop_threads:
            data_msg = Float32MultiArray()
            self.vrpose_data[6] = self.aruco_detector.get_aruco_pixel_distance()
            data_msg.data = list(map(float, self.vrpose_data))
            self.data_publisher.publish(data_msg)
            time.sleep(0.05)  # 20 Hz

    def delta_pose_callback(self, delta_pose_msg):
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

def main():
    rclpy.init()
    try:
        controller = Ur3DeltaPoseController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        controller.shutdown()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
