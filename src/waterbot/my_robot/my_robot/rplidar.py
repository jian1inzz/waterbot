#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from numpy import linspace, inf
from math import degrees
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):
    def __init__(self):
        super().__init__('stretch_scan_filter')

        self.pub = self.create_publisher(
            LaserScan, '/filtered_scan', 10)
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_filter_callback, 10)

        self.get_logger().info("✅ 正在發布 /filtered_scan，僅保留角度 ±135°")

    def scan_filter_callback(self, msg: LaserScan):
        angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        # ---------- 裁切：只保留 ±135° ----------
        new_ranges = [
            r if abs(theta) <= 2.356 else inf
            for r, theta in zip(msg.ranges, angles)
        ]

        # ---------- 發佈 ----------
        msg.ranges = new_ranges
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
