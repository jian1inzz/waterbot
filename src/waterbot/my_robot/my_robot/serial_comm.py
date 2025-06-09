import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, BatteryState
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import serial
import time

# 定義資料格式的常數
FRAME_HEADER = 0xAA
FRAME_TAIL = 0x55

class SerialTwistPublisher(Node):
    def __init__(self):
        super().__init__('serial_twist_publisher')
        self.ser = serial.Serial('/dev/ttyAMA0', 115200)
        
        # 宣告 ROS2 Publisher
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        self.battery_publisher = self.create_publisher(BatteryState, '/battery', 10)
        
        # 訂閱cmd_vel
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
    
    def to_bytes(self, value):
        value = int(value * 1000)
        high = (value >> 8) & 0xFF
        low = value & 0xFF
        return high, low
    
    def twos_complement(self, value, bits):
        if value & (1 << (bits - 1)):
            value -= 1 << bits
        return value
    
    def calc_checksum(self, data):
        checksum = 0
        for b in data:
            checksum ^= b
        return checksum

    def listener_callback(self, msg):
        try:
            while self.ser.in_waiting > 0:
                data = self.ser.read(24)  # 讀取 24 bytes 數據包
                if data[0] == FRAME_HEADER and data[23] == FRAME_TAIL:
                    checksum = self.calc_checksum(data[:22])  # 計算檢查碼
                    if checksum == data[22]:
                        x_speed_raw = (data[2] << 8) | data[3]
                        y_speed_raw = (data[4] << 8) | data[5]
                        z_speed_raw = (data[6] << 8) | data[7]

                        x_speed = self.twos_complement(x_speed_raw, 16) / 1000  # m/s
                        y_speed = self.twos_complement(y_speed_raw, 16) / 1000  # m/s
                        z_speed = self.twos_complement(z_speed_raw, 16) / 1000  # m/s

                        # 加速度 (raw -> m/s²)
                        accel_x_raw = (data[8] << 8) | data[9]
                        accel_y_raw = (data[10] << 8) | data[11]
                        accel_z_raw = (data[12] << 8) | data[13]

                        accel_x = self.twos_complement(accel_x_raw, 16) / 16384 * 9.8  # m/s²
                        accel_y = self.twos_complement(accel_y_raw, 16) / 16384 * 9.8
                        accel_z = self.twos_complement(accel_z_raw, 16) / 16384 * 9.8

                        # 角速度 (raw -> rad/s)
                        gyro_x_raw = (data[14] << 8) | data[15]
                        gyro_y_raw = (data[16] << 8) | data[17]
                        gyro_z_raw = (data[18] << 8) | data[19]

                        gyro_x = self.twos_complement(gyro_x_raw, 16) / 3754.9  # rad/s
                        gyro_y = self.twos_complement(gyro_y_raw, 16) / 3754.9
                        gyro_z = self.twos_complement(gyro_z_raw, 16) / 3754.9

                        # 電壓 (raw -> V)
                        voltage_raw = (data[20] << 8) | data[21]
                        voltage = voltage_raw / 1000.0  # V

                        # --- 打包資料並發佈 ---
                        # IMU 資料
                        imu_msg = Imu()
                        imu_msg.header = Header()
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.linear_acceleration.x = accel_x
                        imu_msg.linear_acceleration.y = accel_y
                        imu_msg.linear_acceleration.z = accel_z
                        imu_msg.angular_velocity.x = gyro_x
                        imu_msg.angular_velocity.y = gyro_y
                        imu_msg.angular_velocity.z = gyro_z
                        self.imu_publisher.publish(imu_msg)

                        # 電池電壓
                        battery_msg = BatteryState()
                        battery_msg.header = Header()
                        battery_msg.header.stamp = self.get_clock().now().to_msg()
                        battery_msg.voltage = voltage
                        self.battery_publisher.publish(battery_msg)

                        # 打印輸出
                        print(f"✅ 收到一筆資料！")
                        print(f"X速度: {x_speed:.3f} m/s, Y速度: {y_speed:.3f} m/s, Z角速度: {z_speed:.3f} m/s")
                        print(f"加速度 -> X: {accel_x:.3f} m/s², Y: {accel_y:.3f} m/s², Z: {accel_z:.3f} m/s²")
                        print(f"角速度 -> X: {gyro_x:.3f} rad/s, Y: {gyro_y:.3f} rad/s, Z: {gyro_z:.3f} rad/s")
                        print(f"電壓: {voltage:.3f} V")
                        print("-" * 50)

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("中斷連線，退出。")
        finally:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = SerialTwistPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
