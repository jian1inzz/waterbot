#!/usr/bin/env python3
# patrol_node.py - ROS 2 高智慧探索巡邏 Node（左右障礙物決策繞行 + 自動回正）
import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import random

FORWARD_V = 0.05     # 基本前進速度 (m/s)
TURN_V = 0.5         # 基本轉向速度 (rad/s)
STRAIGHT_TIME_BASE = 50  # 直行基準時間 (次迴圈)
TURN_TIME_BASE = 20      # 轉向基準時間 (次迴圈)
REALIGN_TIME = 10        # 回正調整時間 (次迴圈)，0.1s * 10 = 1 秒

SAFE_DIST = 0.4  # 前方安全距離 (m)，小於此距離嘗試繞行

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        self.pub_patrol = self.create_publisher(Twist, '/patrol_cmd_vel', 10)  # 發布建議速度，不直接控制 /cmd_vel
        self.create_subscription(Float32, '/obstacle_distance', self.cb_obstacle, 10)
        self.create_subscription(Float32, '/obstacle_left', self.cb_obstacle_left, 10)
        self.create_subscription(Float32, '/obstacle_right', self.cb_obstacle_right, 10)
        self.create_subscription(Float32, '/yolo_angle', self.cb_yolo_angle, 10)

        self.create_timer(0.1, self.patrol_loop)

        self.state = 'straight'  # 狀態：'straight'、'turn'、'realign'
        self.counter = 0         # 計數器控制狀態切換

        # 記憶變數
        self.total_straight_time = 0
        self.last_turn_direction = 1  # 上一次轉向方向 (1:左 -1:右)

        self.obstacle_dist = float('inf')
        self.left_dist = float('inf')
        self.right_dist = float('inf')
        self.yolo_detected = False

        self.get_logger().info("✅ Patrol Node 啟動 (左右決策 + 自動回正)")

    def cb_obstacle(self, msg: Float32):
        self.obstacle_dist = msg.data

    def cb_obstacle_left(self, msg: Float32):
        self.left_dist = msg.data

    def cb_obstacle_right(self, msg: Float32):
        self.right_dist = msg.data

    def cb_yolo_angle(self, msg: Float32):
        self.yolo_detected = True

    def patrol_loop(self):
        twist = Twist()

        if self.state == 'straight':
            if self.obstacle_dist < SAFE_DIST:
                self.state = 'turn'
                self.counter = 0

                # 決定轉向方向
                if self.left_dist > self.right_dist:
                    self.turn_direction = 1
                elif self.right_dist > self.left_dist:
                    self.turn_direction = -1
                else:
                    self.turn_direction = random.choice([-1, 1])

                self.last_turn_direction = self.turn_direction
                self.turn_duration = random.randint(10, TURN_TIME_BASE)
                return

            twist.linear.x = FORWARD_V
            twist.angular.z = 0.0

            self.counter += 1
            self.total_straight_time += 1

            if self.counter >= STRAIGHT_TIME_BASE:
                self.state = 'turn'
                self.counter = 0
                self.turn_direction = random.choice([-1, 1])
                self.last_turn_direction = self.turn_direction
                self.turn_duration = random.randint(10, TURN_TIME_BASE)

        elif self.state == 'turn':
            twist.linear.x = 0.0
            twist.angular.z = self.turn_direction * TURN_V

            self.counter += 1
            if self.counter >= self.turn_duration:
                # 完成轉彎後，先進入回正狀態
                self.state = 'realign'
                self.counter = 0
                self.turn_direction = -self.last_turn_direction  # 反方向校正

        elif self.state == 'realign':
            twist.linear.x = 0.0
            twist.angular.z = self.turn_direction * TURN_V * 0.5  # 校正時轉慢一點

            self.counter += 1
            if self.counter >= REALIGN_TIME:
                self.state = 'straight'
                self.counter = 0
                self.total_straight_time = 0

        # 發布建議速度給 Decision Node 決定是否採用
        self.pub_patrol.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 手動中斷")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
