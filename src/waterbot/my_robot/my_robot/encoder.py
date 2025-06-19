#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import serial
import time
import threading
import queue

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
PORT = '/dev/ttyACM0'
BAUDRATE = 115200

def twos_complement(value, bits):
    if value & (1 << (bits - 1)):
        value -= 1 << bits
    return value

class SerialTwistAndEncoderNode(Node):
    def __init__(self):
        super().__init__('serial_twist_and_encoder_node')

        try:
            self.ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        except serial.SerialException:
            return

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.x_speed_pub = self.create_publisher(Float32, '/x_speed', 10)

        self.send_queue = queue.Queue()
        self.sender_thread = threading.Thread(target=self.send_thread_fn, daemon=True)
        self.sender_thread.start()

        self.buffer = bytearray()
        self.timer = self.create_timer(0.05, self.read_serial)
        self.last_log_time = time.time()

    def cmd_callback(self, msg: Twist):
        packet = self.build_velocity_packet(msg.linear.x, msg.linear.y, msg.angular.z)
        self.send_queue.put(packet)

    def send_thread_fn(self):
        while True:
            try:
                packet = self.send_queue.get(timeout=1)
                self.ser.write(packet)
                # 不顯示發送 log
            except queue.Empty:
                continue

    def float_to_bytes(self, value):
        value = int(value * 1000)
        high = (value >> 8) & 0xFF
        low = value & 0xFF
        return high, low

    def build_velocity_packet(self, linear_x, linear_y, angular_z):
        tx = [0] * 11
        tx[0] = FRAME_HEADER
        tx[1] = 0
        tx[2] = 0
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

    def read_serial(self):
        if self.ser.in_waiting > 0:
            self.buffer += self.ser.read(self.ser.in_waiting)

            while len(self.buffer) >= 24:
                if self.buffer[0] != FRAME_HEADER:
                    self.buffer.pop(0)
                    continue
                if self.buffer[23] != FRAME_TAIL:
                    self.buffer.pop(0)
                    continue

                data = self.buffer[:24]
                self.buffer = self.buffer[24:]

                x_speed_raw = (data[2] << 8) | data[3]
                y_speed_raw = (data[4] << 8) | data[5]
                z_speed_raw = (data[6] << 8) | data[7]

                x_speed = twos_complement(x_speed_raw, 16) / 1000.0
                y_speed = twos_complement(y_speed_raw, 16) / 1000.0
                z_speed = twos_complement(z_speed_raw, 16) / 1000.0

                self.x_speed_pub.publish(Float32(data=x_speed))
                # 不顯示接收 log

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialTwistAndEncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
