#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D

class SerialTwistPublisher(Node):
    def __init__(self):
        super().__init__('serial_twist_publisher')

        # 根據你的板子調整 serial port 路徑
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

    def float_to_bytes(self, value):
        value = int(value * 1000)  # 放大1000倍變成整數
        high = (value >> 8) & 0xFF
        low = value & 0xFF
        return high, low

    def build_velocity_packet(self, linear_x, linear_y, angular_z):
        tx = [0] * 11
        tx[0] = FRAME_HEADER
        tx[1] = 0  # AutoRecharge 預留位
        tx[2] = 0  # 預留位

        xH, xL = self.float_to_bytes(linear_x)
        yH, yL = self.float_to_bytes(linear_y)
        zH, zL = self.float_to_bytes(angular_z)

        tx[3], tx[4] = xH, xL
        tx[5], tx[6] = yH, yL
        tx[7], tx[8] = zH, zL

        tx[9] = self.checksum(tx[:9])
        tx[10] = FRAME_TAIL
        return bytes(tx)

    def checksum(self, data):
        check = 0
        for b in data:
            check ^= b
        return check

    def listener_callback(self, msg):
        packet = self.build_velocity_packet(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z
        )
        self.ser.write(packet)
        self.get_logger().info(
            f'Sent packet: {[hex(b) for b in packet]}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = SerialTwistPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# 👇 加上這行就會自動執行 main()
if __name__ == '__main__':
    main()
