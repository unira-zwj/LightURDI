import sys
import math

from robot_interface.util_interface import RobotArmInterface

sys.path.append("./base_robot_api/RTDE_Python_Client_Library")
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import rtde_control
import rtde_receive


class UR3RobotArmControl(RobotArmInterface):
    def __init__(self, host='192.168.0.1'):
        self.host = host
        self.con = rtde.RTDE(self.host)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.host)
        self.rtde_c = rtde_control.RTDEControlInterface(self.host)
        self.connected = False

    def connect(self):
        self.con.connect()
        if not self.con.is_connected():
            print("Failed to connect to the UR3 robot")
            sys.exit(1)
        else:
            print("Connected to the UR3 robot")
        version = self.con.get_controller_version()
        print(f"Connected to UR3 with controller version: {version}")
        self.current_pose = self.rtde_r.getActualTCPPose()

    def disconnect(self):
        if self.con.is_connected():
            self.servo_stop()
            self.stop_script()
            self.con.disconnect()
            print("Disconnected from the UR3 robot")

    def get_current_pose(self):
        return self.rtde_r.getActualTCPPose()

    def move_to_pose(self, pose, speed=1.0, acceleration=1.0, time=1.0/30, lookahead_time=0.03, gain=1000):
        self.rtde_c.servoL(pose, speed, acceleration, time, lookahead_time, gain)
    
    def servo_stop(self):
        self.rtde_c.servoStop()

    def stop_script(self):
        self.rtde_c.stopScript()
