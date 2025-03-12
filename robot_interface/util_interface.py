from abc import ABC, abstractmethod

class RobotArmInterface(ABC):
    @abstractmethod
    def connect(self):
        """连接到机械臂"""
        pass

    @abstractmethod
    def disconnect(self):
        """断开与机械臂的连接"""
        pass

    @abstractmethod
    def get_current_pose(self):
        """获取机械臂的当前位姿"""
        pass

    @abstractmethod
    def move_to_pose(self, pose, speed=1.0, acceleration=1.0, time=1.0/30, lookahead_time=0.03, gain=1000):
        """移动机械臂到指定的位姿"""
        pass

    @abstractmethod
    def servo_stop(self):
        """停止机械臂的伺服控制"""
        pass

    @abstractmethod
    def stop_script(self):
        """停止机械臂的脚本"""
        pass


class GripperInterface(ABC):
    @abstractmethod
    def open(self):
        pass

    @abstractmethod
    def close(self):
        pass

    @abstractmethod
    def activate(self):
        pass

    @abstractmethod
    def monitor_status(self, target_status):
        pass
