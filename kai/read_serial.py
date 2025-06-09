import serial
import struct
import time

# --- 設定 ---
PORT = '/dev/ttyACM0'
BAUDRATE = 115200
FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D

def twos_complement(value, bits):
    """兩補數轉換成正負數"""
    if value & (1 << (bits - 1)):
        value -= 1 << bits
    return value

def main():
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print(f"連接到 {PORT}，開始接收資料...\n")
    except serial.SerialException as e:
        print(f"無法開啟 {PORT}: {e}")
        return

    buffer = bytearray()

    try:
        while True:
            if ser.in_waiting > 0:
                buffer += ser.read(ser.in_waiting)

                while len(buffer) >= 24:
                    # 找到Frame Header
                    if buffer[0] != FRAME_HEADER:
                        buffer.pop(0)
                        continue

                    # 確保長度夠
                    if buffer[23] != FRAME_TAIL:
                        buffer.pop(0)
                        continue

                    data = buffer[:24]
                    buffer = buffer[24:]

                    # --- 開始解析 ---
                    # X/Y/Z 速度
                    x_speed_raw = (data[2] << 8) | data[3]
                    y_speed_raw = (data[4] << 8) | data[5]
                    z_speed_raw = (data[6] << 8) | data[7]

                    x_speed = twos_complement(x_speed_raw, 16) / 1000  # m/s
                    y_speed = twos_complement(y_speed_raw, 16) / 1000  # m/s
                    z_speed = twos_complement(z_speed_raw, 16) / 1000  # m/s

                    # 加速度 (raw -> m/s²)
                    accel_x_raw = (data[8] << 8) | data[9]
                    accel_y_raw = (data[10] << 8) | data[11]
                    accel_z_raw = (data[12] << 8) | data[13]

                    accel_x = twos_complement(accel_x_raw, 16) / 16384 * 9.8  # m/s²
                    accel_y = twos_complement(accel_y_raw, 16) / 16384 * 9.8
                    accel_z = twos_complement(accel_z_raw, 16) / 16384 * 9.8

                    # 角速度 (raw -> rad/s)
                    gyro_x_raw = (data[14] << 8) | data[15]
                    gyro_y_raw = (data[16] << 8) | data[17]
                    gyro_z_raw = (data[18] << 8) | data[19]

                    gyro_x = twos_complement(gyro_x_raw, 16) / 3754.9  # rad/s
                    gyro_y = twos_complement(gyro_y_raw, 16) / 3754.9
                    gyro_z = twos_complement(gyro_z_raw, 16) / 3754.9

                    # 電壓 (raw -> V)
                    voltage_raw = (data[20] << 8) | data[21]
                    voltage = voltage_raw / 1000.0  # V

                    # --- 印出結果 ---
                    print("✅ 收到一筆資料！")
                    print(f"X速度: {x_speed:.3f} m/s, Y速度: {y_speed:.3f} m/s, Z角速度: {z_speed:.3f} m/s")
                    print(f"加速度 -> X: {accel_x:.3f} m/s², Y: {accel_y:.3f} m/s², Z: {accel_z:.3f} m/s²")
                    print(f"角速度 -> X: {gyro_x:.3f} rad/s, Y: {gyro_y:.3f} rad/s, Z: {gyro_z:.3f} rad/s")
                    print(f"電壓: {voltage:.3f} V")
                    print("-" * 50)

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("中斷連線，退出。")
    finally:
        ser.close()

if __name__ == '__main__':
    main()
