#!/usr/bin/env python3
# obstacle_avoid_node.py - ROS 2 障礙物處理 Node（前方 ±15°，左右 ±45° 擴大偵測）
import math, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

SAFE_DIST = 0.5  # 安全距離 (m)，小於這個距離認為有障礙物

class ObstacleAvoidNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoid_node')

        # 訂閱雷達掃描資料
        self.create_subscription(LaserScan, '/filtered_scan', self.cb_scan, 10)
        # 發布前方、左側、右側障礙物距離資訊
        self.pub_front = self.create_publisher(Float32, '/obstacle_distance', 10)
        self.pub_left  = self.create_publisher(Float32, '/obstacle_left', 10)
        self.pub_right = self.create_publisher(Float32, '/obstacle_right', 10)

        self.get_logger().info("✅ Obstacle Avoid Node 啟動 (前 ±15°，左右 ±45°)")

    def cb_scan(self, scan: LaserScan):
        inc = scan.angle_increment
        total_ranges = len(scan.ranges)

        # 計算索引範圍
        n15 = int(math.radians(15) / inc)
        n45 = int(math.radians(45) / inc)
        n90 = int(math.radians(90) / inc)

        # 前方 ±15°
        front_ranges = scan.ranges[:n15] + scan.ranges[-n15:]
        front_dist = min(front_ranges)

        # 左側 45° ~ 135°
        left_start = (total_ranges // 4) - n45
        left_end   = (total_ranges // 4) + n45
        left_ranges = scan.ranges[left_start:left_end]
        left_dist = min(left_ranges)

        # 右側 -135° ~ -45°
        right_start = (3 * total_ranges // 4) - n45
        right_end   = (3 * total_ranges // 4) + n45
        right_ranges = scan.ranges[right_start:right_end]
        right_dist = min(right_ranges)

        # 發布各方向的最近障礙物距離
        msg_front = Float32(); msg_front.data = front_dist
        msg_left  = Float32(); msg_left.data  = left_dist
        msg_right = Float32(); msg_right.data = right_dist

        self.pub_front.publish(msg_front)
        self.pub_left.publish(msg_left)
        self.pub_right.publish(msg_right)

        # Log 障礙物情況
        if front_dist < SAFE_DIST:
            self.get_logger().info(f"⚠️ 前方障礙物 距離：{front_dist:.2f} m")
        if left_dist < SAFE_DIST:
            self.get_logger().info(f"⚠️ 左側障礙物 距離：{left_dist:.2f} m")
        if right_dist < SAFE_DIST:
            self.get_logger().info(f"⚠️ 右側障礙物 距離：{right_dist:.2f} m")


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
