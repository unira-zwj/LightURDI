# gripper_control.py

from pymodbus.client import ModbusSerialClient as ModbusClient
import time
from robot_interface.util_interface import GripperInterface

class Robotiq85(GripperInterface):
    def __init__(self, MODBUS_PORT='/dev/ttyUSB0', BAUDRATE=115200) -> None:
        self.SPEED = 20
        self.FORCE = 20
        self.client = ModbusClient(
            port=MODBUS_PORT,
            baudrate=BAUDRATE,
            timeout=1,
            parity='N',
            stopbits=1,
            bytesize=8
        )

    def activate(self, slave=9):
        """激活夹爪"""
        # 尝试连接到 Modbus 服务器
        if not self.client.connect():
            print("Failed to connect to the Modbus server.")
            return False

        try:
            # 假设激活通过写入寄存器地址 1000（0x03E8）
            address = 0x03E8
            values = [0x0000, 0x0000, 0x0000]  # 激活值
            response = self.client.write_registers(address, values, slave=slave)

            if response.isError():
                print("Error activating gripper.")
                return False

            status_address = 0x07D0  # 读取夹爪状态的寄存器地址
            while True:
                response = self.client.read_holding_registers(status_address, 1, slave=slave)
                if response.isError():
                    print("Error reading gripper status.")
                    return False

                status = response.registers[0]
                # 检查激活状态（根据实际设备响应修改条件）
                if status == 0x0000:
                    # print("Gripper activated.")
                    break
                else:
                    print("Waiting for activation...")
                    time.sleep(0.5)  # 等待后再轮询
            return True
        finally:
            self.client.close()

    def open(self, slave=9):
        """打开夹爪"""
        if not self.client.connect():
            print("Failed to connect to the Modbus server.")
            return

        try:
            # if not self.activate(slave):
            #     print("Failed to activate gripper.")
            #     return

            # 打开夹爪的命令
            address = 0x03E8
            command = 0x0900  # 打开命令，设置必要的位
            position = 0x0000  # 打开位置
            speed_force = (200 << 8) | self.FORCE  # 将速度和力组合成一个寄存器
            values = [command, position, speed_force]

            response = self.client.write_registers(address, values, slave=slave)
            if response.isError():
                print("Error sending open command.")
                return

            time.sleep(0.8)  # 等待命令执行
        finally:
            self.client.close()

    def close(self, slave=9):
        """关闭夹爪"""
        if not self.client.connect():
            print("Failed to connect to the Modbus server.")
            return

        try:
            if not self.activate(slave):
                print("Failed to activate gripper.")
                return

            # 关闭夹爪的命令
            address = 0x03E8
            command = 0x0900  # 关闭命令，设置必要的位
            position = 0x00FF  # 关闭位置
            speed_force = (self.SPEED << 8) | self.FORCE  # 将速度和力组合成一个寄存器
            values = [command, position, speed_force]

            response = self.client.write_registers(address, values, slave=slave)
            if response.isError():
                print("Error sending close command.")
                return

            time.sleep(0.8)  # 等待命令执行
        finally:
            self.client.close()

    def monitor_status(self, target_status, slave=9):
        """监控夹爪是否达到目标状态"""
        if not self.client.connect():
            print("Failed to connect to the Modbus server.")
            return False

        try:
            status_address = 0x07D0  # 读取夹爪状态的寄存器地址
            while True:
                response = self.client.read_holding_registers(status_address, 1, slave=slave)
                if response.isError():
                    print("Error reading gripper status.")
                    return False

                status = response.registers[0]
                # 根据具体设备的状态位解码
                if target_status == 'closed':
                    # 替换为实际设备的状态位条件
                    if (status & 0x0001):  # 假设 0x0001 位表示已关闭
                        print("Gripper has closed.")
                        break
                elif target_status == 'open':
                    if (status & 0x0002):  # 假设 0x0002 位表示已打开
                        print("Gripper has opened.")
                        break
                else:
                    print("Unknown target status.")
                    return False
                time.sleep(0.5)  # 等待后再轮询
            return True
        finally:
            self.client.close()

# 如果需要单独测试，可以添加如下内容
if __name__ == '__main__':
    gripper = Robotiq85()
    gripper.open()
    time.sleep(2)
    gripper.close()
