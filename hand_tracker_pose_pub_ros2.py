import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import numpy as np

import utils.triad_openvr as triad_openvr


class VRPosePublisher(Node):
    def __init__(self, tracker_name="tracker_1", topic_name="/trackerA/pose", frame_id="world", fps=200):
        """
        Initialize the VRPosePublisher node.

        :param tracker_name: Name of the VR device to track.
        :param topic_name: ROS2 topic name for publishing pose information.
        :param frame_id: Reference coordinate frame ID.
        :param fps: Publishing rate (Hz).
        """
        super().__init__('vr_pose_publisher')

        # Initialize triad_openvr
        self.vr = triad_openvr.triad_openvr()
        self.vr.print_discovered_objects()

        self.tracker_name = tracker_name
        self.topic_name = topic_name
        self.frame_id = frame_id
        self.fps = fps
        self.interval = 1.0 / self.fps

        # Create publisher
        self.pose_pub = self.create_publisher(PoseStamped, self.topic_name, 10)

        # Initialize timer
        timer_period = self.interval  # seconds
        self.timer = self.create_timer(timer_period, self.publish_pose)

        # Create PoseStamped message
        self.pose_msg = PoseStamped()
        self.pose_msg.header.frame_id = self.frame_id

        # Initialize previous Euler angles (for unwrapping to prevent jumps)
        self.prev_euler = [0.0, 0.0, 0.0]

        # Check if the tracker is available
        if self.tracker_name in self.vr.devices:
            self.tracker_available = True
            self.get_logger().info(f"Successfully connected to tracker: {self.tracker_name}")
        else:
            self.tracker_available = False
            self.get_logger().warn(f"Tracker '{self.tracker_name}' not found. Please ensure it is active.")

    def unwrap_angle(self, current, prev):
        """
        Ensure angle continuity to prevent jumps.

        :param current: Current angle (radians).
        :param prev: Previous accumulated angle (radians).
        :return: Accumulated current angle (radians).
        """
        delta = current - prev
        delta = (delta + math.pi) % (2 * math.pi) - math.pi  # Adjust to [-pi, pi] range
        return prev + delta

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion.

        :param roll: Roll angle (radians).
        :param pitch: Pitch angle (radians).
        :param yaw: Yaw angle (radians).
        :return: Quaternion (qx, qy, qz, qw).
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - \
             math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + \
             math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - \
             math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + \
             math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw

    def get_pose(self):
        """
        Retrieve the current pose of the tracker.

        :return: Processed position (meters) and orientation (quaternion).
        """
        if not self.tracker_available:
            self.get_logger().warn("Tracker not available. Move tracker to active...")
            return None, None

        try:
            pose = self.vr.devices[self.tracker_name].get_pose_euler()
            if pose is None:
                self.get_logger().warn("Received None pose data.")
                return None, None

            # Assume pose returns [x, y, z, roll, pitch, yaw]
            x = round(pose[0], 4)  # meters
            y = round(pose[2], 4)
            z = round(pose[1], 4)

            # Convert degrees to radians
            roll = math.radians(pose[3])
            pitch = math.radians(pose[5])
            yaw = math.radians(pose[4])

            # Prevent angle jumps
            roll = self.unwrap_angle(roll, self.prev_euler[0])
            pitch = self.unwrap_angle(pitch, self.prev_euler[1])
            yaw = self.unwrap_angle(yaw, self.prev_euler[2])

            # Update previous angles
            self.prev_euler = [roll, pitch, yaw]

            # Convert to quaternion
            qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

            return (x, y, z), (qx, qy, qz, qw)

        except KeyError:
            self.get_logger().warn("Tracker not available. Move tracker to active...")
            return None, None
        except Exception as e:
            self.get_logger().error(f"Error getting pose: {e}")
            return None, None

    def publish_pose(self):
        """
        Retrieve and publish the pose to the ROS2 topic.
        """
        position, orientation = self.get_pose()
        if position and orientation:
            self.pose_msg.header.stamp = self.get_clock().now().to_msg()
            self.pose_msg.pose.position.x = position[0]
            self.pose_msg.pose.position.y = position[1]
            self.pose_msg.pose.position.z = position[2]
            self.pose_msg.pose.orientation.x = self.prev_euler[0] # roll
            self.pose_msg.pose.orientation.y = self.prev_euler[1] # pitch
            self.pose_msg.pose.orientation.z = self.prev_euler[2] # yaw
            self.pose_msg.pose.orientation.w = 0.0

            self.pose_pub.publish(self.pose_msg)

            # Log pose information
            self.get_logger().info(
                f"Position (m): ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) "
                f"Orientation (rad): ({self.prev_euler[0]:.3f}, {self.prev_euler[1]:.3f}, {self.prev_euler[2]:.3f})"
            )
        else:
            self.get_logger().debug("No pose data to publish.")


def main(args=None):
    """
    Main function to initialize the ROS2 node and keep it spinning.
    """
    rclpy.init(args=args)

    # Create VRPosePublisher node
    vr_publisher = VRPosePublisher(
        tracker_name="tracker_1",
        topic_name="/trackerA/pose",
        frame_id="world",
        fps=30
    )

    try:
        # Spin the node to keep it active
        rclpy.spin(vr_publisher)
    except KeyboardInterrupt:
        vr_publisher.get_logger().info("Shutting down VRPosePublisher.")
    finally:
        # Destroy the node explicitly
        vr_publisher.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
