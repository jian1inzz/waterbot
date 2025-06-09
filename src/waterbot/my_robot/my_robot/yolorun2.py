#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

FORWARD_V = 0.05
ANG_KP = 0.4
TARGET_DIST = 0.3
ANGLE_GATE = math.radians(3)
IGNORE_ANGLE_WIDTH = math.radians(10)

class YoloFollowNode(Node):
    def __init__(self):
        super().__init__('yolo_follow_node')

        self.angle_deg = None
        self.front_dist = float('inf')

        self.create_subscription(Float32, '/yolo_angle', self.cb_angle, 10)
        self.create_subscription(LaserScan, '/filtered_scan', self.cb_scan, 10)
        self.pub_suggest = self.create_publisher(Twist, '/yolo_cmd_vel', 10)
        self.pub_distance = self.create_publisher(Float32, '/yolo_distance', 10)  # 新增距離發布

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("✅ YOLO 跟隨建議速度 Node 啟動")

    def cb_angle(self, msg: Float32):
        self.angle_deg = msg.data

    def cb_scan(self, scan: LaserScan):
        inc = scan.angle_increment
        angles = [scan.angle_min + i * inc for i in range(len(scan.ranges))]

        filtered_ranges = []
        for r, angle in zip(scan.ranges, angles):
            if self.angle_deg is not None:
                target_angle_rad = math.radians(self.angle_deg)
                if abs(angle - target_angle_rad) <= IGNORE_ANGLE_WIDTH:
                    filtered_ranges.append(float('inf'))
                    continue
            filtered_ranges.append(r)

        n15 = int(math.radians(15) / inc)
        self.front_dist = min(filtered_ranges[:n15] + filtered_ranges[-n15:])

        # 發布估算的目標距離
        self.pub_distance.publish(Float32(data=self.front_dist))

    def control_loop(self):
        twist = Twist()

        if self.angle_deg is not None:
            if self.front_dist < TARGET_DIST:
                self.get_logger().info(f"🛑 測距停車建議，距離 {self.front_dist:.2f}m")
            else:
                ang_err = math.radians(self.angle_deg)
                ang_abs = abs(ang_err)

                lin_v = FORWARD_V * max(0.3, 1.0 - ang_abs * 2.0)
                ang_v = ANG_KP * ang_err

                if ang_abs < ANGLE_GATE:
                    ang_v = 0.0

                twist.linear.x = lin_v
                twist.angular.z = ang_v

                self.get_logger().info(f"🎯 跟隨建議 → 角度: {self.angle_deg:+.1f}° 距離: {self.front_dist:.2f}m")
        else:
            self.get_logger().info("🛑 沒有目標，建議停止")

        self.pub_suggest.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = YoloFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 手動中斷")
    finally:
        node.pub_suggest.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
