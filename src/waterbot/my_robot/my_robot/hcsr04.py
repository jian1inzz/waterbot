#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

import RPi.GPIO as GPIO
import time
import os
import signal
import threading

TRIG = 23  # TRIG GPIO 腳位號碼
ECHO = 24  # ECHO GPIO 腳位號碼

def monitor_parent():
    """🛡️ 當父進程被終止，這個子進程也會自動退出"""
    ppid = os.getppid()
    while True:
        if os.getppid() != ppid:
            print("🔴 父進程已死亡，終止超音波節點")
            os.kill(os.getpid(), signal.SIGINT)
        time.sleep(1)

class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')

        # 初始化 GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)

        self.enabled = False
        self.last_log_time = time.time()

        self.create_subscription(Bool, 'hcsr04_enable', self.cb_enable, 10)
        self.publisher_ = self.create_publisher(Float32, 'ultrasonic_distance', 10)
        self.timer = self.create_timer(0.072, self.timer_callback)

        self.get_logger().info("🔧 超音波距離感測器已啟動")

    def cb_enable(self, msg: Bool):
        self.enabled = msg.data
        if not self.enabled:
            self.get_logger().info("🛑 停止發佈超音波距離")

    def timer_callback(self):
        if not self.enabled:
            return

        distance = self.get_distance()
        if distance is not None:
            msg = Float32()
            msg.data = distance
            self.publisher_.publish(msg)

            # ✅ 每 1 秒印一次距離
            now = time.time()
            if now - self.last_log_time >= 1.0:
                self.get_logger().info(f'📏 距離: {distance:.2f} cm')
                self.last_log_time = now

    def get_distance(self):
        GPIO.output(TRIG, True)
        time.sleep(0.00002)
        GPIO.output(TRIG, False)

        start_time = time.time()
        timeout = start_time + 0.015
        while GPIO.input(ECHO) == 0:
            start_time = time.time()
            if start_time > timeout:
                return None

        end_time = time.time()
        timeout = end_time + 0.015
        while GPIO.input(ECHO) == 1:
            end_time = time.time()
            if end_time > timeout:
                return None

        duration = end_time - start_time
        distance = duration * 17150
        return round(distance, 2)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    # 啟動父進程監控
    threading.Thread(target=monitor_parent, daemon=True).start()

    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
