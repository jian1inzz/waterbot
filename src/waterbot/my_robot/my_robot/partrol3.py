#!/usr/bin/env python3
# patrol_node.py - 巡邏行為建議節點（不處理避障）

import math, rclpy, time, random
from rclpy.node import Node
from geometry_msgs.msg import Twist

FORWARD_V = 0.05     # 巡邏直行速度
TURN_V    = 0.1      # 巡邏轉彎角速度

STRAIGHT_TIME_BASE = 50  # 直走持續次數（0.3s 為一單位）
TURN_TIME_BASE     = 20  # 轉彎最大次數（隨機）
POST_TURN_DELAY    = 8   # 轉彎完暫停步數

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        self.pub = self.create_publisher(Twist, '/patrol_cmd_vel', 10)
        self.timer = self.create_timer(0.3, self.loop)

        self.state = 'straight'
        self.counter = 0
        self.turn_dir = random.choice([-1, 1])
        self.turn_duration = random.randint(10, TURN_TIME_BASE)
        self.pause_counter = 0

        self.get_logger().info("✅ 巡邏節點啟動（不含避障）")

    def loop(self):
        twist = Twist()

        if self.state == 'straight':
            twist.linear.x = FORWARD_V
            self.counter += 1
            if self.counter >= STRAIGHT_TIME_BASE:
                self.state = 'turn'
                self.counter = 0
                self.turn_dir = random.choice([-1, 1])
                self.turn_duration = random.randint(10, TURN_TIME_BASE)
                self.get_logger().info(f"🔄 轉彎開始（方向：{self.turn_dir}）")

        elif self.state == 'turn':
            twist.angular.z = self.turn_dir * TURN_V
            self.counter += 1
            if self.counter >= self.turn_duration:
                self.state = 'pause'
                self.counter = 0
                self.pause_counter = 0
                self.get_logger().info("⏸️ 轉彎完成，暫停中")

        elif self.state == 'pause':
            self.pause_counter += 1
            if self.pause_counter >= POST_TURN_DELAY:
                self.state = 'straight'
                self.counter = 0
                self.get_logger().info("▶️ 繼續直走")
            # twist 維持 0

        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🚫 手動中斷")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
