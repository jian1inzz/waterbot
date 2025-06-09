#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

# --------- 參數設定 ---------
SAFE_DIST_FRONT = 0.4       # 前方安全距離 (m)，小於此距離直接停車（除非正在接近目標）
SAFE_DIST_SIDE = 0.4        # 左右安全距離 (m)，小於此距離視為左右卡死
TARGET_NEAR_DIST = 0.45     # 目標距離小於這個值時，優先允許靠近 YOLO 目標

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        # 訂閱行為 Node 的建議速度
        self.create_subscription(Twist, '/yolo_cmd_vel', self.cb_yolo_cmd, 10)
        self.create_subscription(Twist, '/patrol_cmd_vel', self.cb_patrol_cmd, 10)

        # 訂閱障礙物資訊
        self.create_subscription(Float32, '/obstacle_distance', self.cb_obstacle_front, 10)
        self.create_subscription(Float32, '/obstacle_left', self.cb_obstacle_left, 10)
        self.create_subscription(Float32, '/obstacle_right', self.cb_obstacle_right, 10)
        self.create_subscription(Float32, '/yolo_distance', self.cb_yolo_distance, 10)

        # 發布最終控制速度到 /cmd_vel
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # 儲存狀態資料
        self.yolo_cmd = Twist()         # YOLO 模組的建議速度
        self.patrol_cmd = Twist()       # 巡邏模組的建議速度
        self.front_dist = float('inf')  # 前方障礙物最近距離
        self.left_dist = float('inf')   # 左側障礙物最近距離
        self.right_dist = float('inf')  # 右側障礙物最近距離
        self.target_dist = float('inf') # 目標（YOLO 偵測）距離

        # 每 0.1 秒執行一次決策邏輯
        self.create_timer(0.1, self.decision_loop)
        self.get_logger().info("✅ Decision Node 啟動 (完整優化版)")

    # --------- Callback 區塊 ---------
    def cb_yolo_cmd(self, msg: Twist):
        self.yolo_cmd = msg

    def cb_patrol_cmd(self, msg: Twist):
        self.patrol_cmd = msg

    def cb_obstacle_front(self, msg: Float32):
        self.front_dist = msg.data

    def cb_obstacle_left(self, msg: Float32):
        self.left_dist = msg.data

    def cb_obstacle_right(self, msg: Float32):
        self.right_dist = msg.data

    def cb_yolo_distance(self, msg: Float32):
        self.target_dist = msg.data

    # --------- 決策邏輯主流程 ---------
    def decision_loop(self):
        twist = Twist()  # 預設停車指令

        # 1️⃣ 前方障礙物檢查
        if self.front_dist < SAFE_DIST_FRONT:
            if self.target_dist < TARGET_NEAR_DIST:
                # 目標已接近，允許靠近，不停車
                self.get_logger().info(f"🎯 正在接近目標 (目標距離 {self.target_dist:.2f} m, 前方障礙 {self.front_dist:.2f} m)")
            else:
                # 無目標或目標太遠，停車避免碰撞
                self.get_logger().info(f"🛑 停車：前方障礙物 {self.front_dist:.2f} m，未接近目標")
                self.pub_cmd.publish(twist)
                return

        # 2️⃣ 左右障礙物檢查 → 若兩側都無法通過直接停車
        if self.left_dist < SAFE_DIST_SIDE and self.right_dist < SAFE_DIST_SIDE:
            self.get_logger().info(f"🛑 停車：左右兩側皆無法通過 (左 {self.left_dist:.2f} m, 右 {self.right_dist:.2f} m)")
            self.pub_cmd.publish(twist)
            return

        # 3️⃣ 優先根據目標距離決定行為
        if self.target_dist < TARGET_NEAR_DIST:
            # 優先跟隨 YOLO 目標
            if self.yolo_cmd.linear.x != 0.0 or self.yolo_cmd.angular.z != 0.0:
                self.get_logger().info("🎯 優先跟隨 YOLO 目標 (目標接近)")
                self.pub_cmd.publish(self.yolo_cmd)
                return
        else:
            # 目標較遠，進入巡邏模式
            if self.patrol_cmd.linear.x != 0.0 or self.patrol_cmd.angular.z != 0.0:
                self.get_logger().info("🚗 目標遠 → 進入巡邏模式")
                self.pub_cmd.publish(self.patrol_cmd)
                return

        # 4️⃣ 無任何行為指令 → 停止等待
        self.get_logger().info("🛑 無行為指令，停止等待")
        self.pub_cmd.publish(twist)

# --------- 主程式入口 ---------
def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 手動中斷程式")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
