#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
import os, signal, threading
import time


SAFE_DIST = 0.4     # 安全距離 (m)
MIN_VALID = 0.15     # 最小有效距離（過濾雜訊）

def monitor_parent():
    """🛡️ 當父進程被終止，這個子進程也會自動退出"""
    ppid = os.getppid()
    while True:
        if os.getppid() != ppid:
            print("🔴 父進程已死亡，終止 YOLO 指令節點")
            os.kill(os.getpid(), signal.SIGINT)
        time.sleep(1)

class ObstacleAvoidNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoid_node')

        self.hcsr04_enabled = False  # ✅ 超音波啟動旗標
        self.create_subscription(Bool, '/hcsr04_enable', self.cb_hcsr04, 10)
        self.create_subscription(LaserScan, '/filtered_scan', self.cb_scan, 10)

        # 五區 Publisher
        self.pub_front        = self.create_publisher(Float32, '/obstacle_distance', 10)
        self.pub_left_front   = self.create_publisher(Float32, '/obstacle_left_front', 10)
        self.pub_right_front  = self.create_publisher(Float32, '/obstacle_right_front', 10)
        self.pub_left         = self.create_publisher(Float32, '/obstacle_left', 10)
        self.pub_right        = self.create_publisher(Float32, '/obstacle_right', 10)

        self.get_logger().info("✅ Obstacle Avoid Node 啟動 (五區分辨，±20° 前方)")

    def cb_hcsr04(self, msg: Bool):
        self.hcsr04_enabled = msg.data

    def filter_ranges(self, angles, ranges, min_angle, max_angle):
        """選取角度範圍內，且距離有效的點"""
        return [
            r for a, r in zip(angles, ranges)
            if min_angle <= a <= max_angle and math.isfinite(r) and r >= MIN_VALID
        ]

    def cb_scan(self, scan: LaserScan):
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        ranges = scan.ranges
        total = len(ranges)

        # 每個資料點對應角度
        angles = [angle_min + i * angle_increment for i in range(total)]

        # ✅ 五區角度範圍（單位：rad）
        front_valid =       self.filter_ranges(angles, ranges, -math.radians(20),  math.radians(20))
        left_front_valid =  self.filter_ranges(angles, ranges,  math.radians(20),  math.radians(60))
        right_front_valid = self.filter_ranges(angles, ranges, -math.radians(60), -math.radians(20))
        left_valid =        self.filter_ranges(angles, ranges,  math.radians(60),  math.radians(135))
        right_valid =       self.filter_ranges(angles, ranges, -math.radians(135), -math.radians(60))

        # 計算最小距離
        front_dist       = min(front_valid)       if front_valid else float('inf')
        left_front_dist  = min(left_front_valid)  if left_front_valid else float('inf')
        right_front_dist = min(right_front_valid) if right_front_valid else float('inf')
        left_dist        = min(left_valid)        if left_valid else float('inf')
        right_dist       = min(right_valid)       if right_valid else float('inf')

        # ✅ 發布距離（啟用超音波時略過前方三區）
        if not self.hcsr04_enabled:
            self.pub_front.publish(Float32(data=front_dist))
            self.pub_left_front.publish(Float32(data=left_front_dist))
            self.pub_right_front.publish(Float32(data=right_front_dist))
        else:
            self.pub_front.publish(Float32(data=float('inf')))
            self.pub_left_front.publish(Float32(data=float('inf')))
            self.pub_right_front.publish(Float32(data=float('inf')))

        self.pub_left.publish(Float32(data=left_dist))
        self.pub_right.publish(Float32(data=right_dist))

        # 警告（僅在未啟動超音波時檢查前方）
        if not self.hcsr04_enabled:
            if front_dist < SAFE_DIST:
                self.get_logger().warn(f"⚠️ 前方障礙物：{front_dist:.2f} m")
            if left_front_dist < SAFE_DIST:
                self.get_logger().warn(f"⚠️ 左前方障礙物：{left_front_dist:.2f} m")
            if right_front_dist < SAFE_DIST:
                self.get_logger().warn(f"⚠️ 右前方障礙物：{right_front_dist:.2f} m")

        if left_dist < SAFE_DIST:
            self.get_logger().warn(f"⚠️ 左側障礙物：{left_dist:.2f} m")
        if right_dist < SAFE_DIST:
            self.get_logger().warn(f"⚠️ 右側障礙物：{right_dist:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 手動中斷")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
