#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import serial
import time
import threading
import queue
from collections import deque          # ① 新增
from struct import unpack
import signal
import os

FRAME_HEADER = 0x7B
FRAME_TAIL   = 0x7D
PORT     = '/dev/ttyACM0'
BAUDRATE = 115200

ACC_SCALE = 9.80665 / 16384.0  # ±2 g → 16384

# ② 非線性電壓-百分比曲線（可依實測調）
BAT_TABLE = [
    (24.5, 100), (24.0, 90), (23.5, 80),
    (23.0, 70),  (22.5, 60), (22.0, 40),
    (21.5, 20),  (21.0, 10), (20.5, 5),
    (20.0, 0)
]

def estimate_percent(v: float) -> int:
    for i in range(len(BAT_TABLE) - 1):
        vh, ph = BAT_TABLE[i]
        vl, pl = BAT_TABLE[i + 1]
        if v >= vl:
            return round(pl + (v - vl) / (vh - vl) * (ph - pl))
    return 0

def twos_complement(value, bits):
    if value & (1 << (bits - 1)):
        value -= 1 << bits
    return value
def monitor_parent():
    """🛡️ 當父進程被終止，這個子節點也自動退出"""
    ppid = os.getppid()
    while True:
        if os.getppid() != ppid:
            print("🔴 父進程已死亡，終止串列節點")
            os.kill(os.getpid(), signal.SIGINT)
        time.sleep(1)

class SerialTwistAndEncoderNode(Node):
    def __init__(self):
        super().__init__('serial_twist_and_encoder_node')

        try:
            self.ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"❌ 無法開啟序列埠 {PORT}: {e}")
            return

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.x_speed_pub = self.create_publisher(Float32, '/x_speed', 10)
        self.voltage_pub = self.create_publisher(Float32, '/battery_voltage', 10)
        self.percent_pub = self.create_publisher(Float32, '/battery_percent', 10)  # ③ 新增
        self.imu_pub     = self.create_publisher(Imu, '/imu', 10)

        self.volt_window = deque(maxlen=5)   # ④ 新增：電壓滑動平均用

        self.send_queue = queue.Queue()
        self.sender_thread = threading.Thread(target=self.send_thread_fn, daemon=True)
        self.sender_thread.start()

        self.buffer = bytearray()
        self.timer  = self.create_timer(0.05, self.read_serial)
        self.last_log_time = time.time()
        # self.get_logger().info('✅ Serial Twist + Encoder Node 啟動（指令 + 回報速度 + IMU + 電壓）')

    def cmd_callback(self, msg: Twist):
        packet = self.build_velocity_packet(msg.linear.x, msg.linear.y, msg.angular.z)
        self.send_queue.put(packet)

    def send_thread_fn(self):
        while True:
            try:
                packet = self.send_queue.get(timeout=1)
                self.ser.write(packet)
                # self.get_logger().info(f'📤 發送島包: {[hex(b) for b in packet]}')
            except queue.Empty:
                continue

    def float_to_bytes(self, value):
        iv = int(value * 1000)
        return (iv >> 8) & 0xFF, iv & 0xFF

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

            # --- 解碼速度 ---
            x_speed_raw = (data[2] << 8) | data[3]
            y_speed_raw = (data[4] << 8) | data[5]
            z_speed_raw = (data[6] << 8) | data[7]

            x_speed = twos_complement(x_speed_raw, 16) / 1000.0
            y_speed = twos_complement(y_speed_raw, 16) / 1000.0
            z_speed = twos_complement(z_speed_raw, 16) / 1000.0
            self.x_speed_pub.publish(Float32(data=x_speed))

            # --- 解碼 IMU 加速度 ---
            acc_x = twos_complement((data[8] << 8) | data[9], 16)
            acc_y = twos_complement((data[10] << 8) | data[11], 16)
            acc_z = twos_complement((data[12] << 8) | data[13], 16)

            # --- 解碼 IMU 角速度 ---
            gyr_x = twos_complement((data[14] << 8) | data[15], 16)
            gyr_y = twos_complement((data[16] << 8) | data[17], 16)
            gyr_z = twos_complement((data[18] << 8) | data[19], 16)

            # --- 解碼電壓 ---
            voltage_raw = (data[20] << 8) | data[21]
            voltage = voltage_raw / 1000.0
            self.voltage_pub.publish(Float32(data=voltage))

            # --- 平滑 + 百分比估算 ---
            self.volt_window.append(voltage)
            v_avg = sum(self.volt_window) / len(self.volt_window)
            percent = float(estimate_percent(v_avg))
            self.percent_pub.publish(Float32(data=percent))

            # --- 發布 IMU ---
            imu_msg = Imu()
            imu_msg.linear_acceleration.x = acc_x * ACC_SCALE
            imu_msg.linear_acceleration.y = acc_y * ACC_SCALE
            imu_msg.linear_acceleration.z = acc_z * ACC_SCALE

            imu_msg.angular_velocity.x = gyr_x / 1000.0
            imu_msg.angular_velocity.y = gyr_y / 1000.0
            imu_msg.angular_velocity.z = gyr_z / 1000.0

            self.imu_pub.publish(imu_msg)

            # --- 每秒顯示一次 log ---
            # now = time.time()
            # if now - self.last_log_time >= 1.0:
            #     self.get_logger().info(f"📦 UART 島包: {[hex(b) for b in data]}")
            #     self.get_logger().info(
            #         f"📅 速度: X={x_speed:.3f}, Y={y_speed:.3f}, Z={z_speed:.3f} m/s ｜"
            #         f"電量: {percent:.0f}% ｜"
            #         f"IMU 加速度換算: [{acc_x * ACC_SCALE:.3f}, {acc_y * ACC_SCALE:.3f}, {acc_z * ACC_SCALE:.3f}] m/s² ｜"
            #         f"IMU 角速度: [{gyr_x / 1000.0:.3f}, {gyr_y / 1000.0:.3f}, {gyr_z / 1000.0:.3f}] rad/s"
            #     )
            #     self.last_log_time = now

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    threading.Thread(target=monitor_parent, daemon=True).start()  # ← ✅加這行
    rclpy.init(args=args)
    node = SerialTwistAndEncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # node.get_logger().info("🚫 手動中斷")
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
