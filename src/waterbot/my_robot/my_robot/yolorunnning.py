#!/usr/bin/env python3
import math, time, rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ----------- 參數 ----------
FORWARD_V     = 0.03  # 🔄 降低基礎速度
ANG_KP        = 0.4
HC_STOP_CM    = 8.0   # 🔄 提高停止距離閾值
ANGLE_GATE    = math.radians(2)  # 🔄 縮小角度閾值
EXPIRE_SEC    = 4.0
REACTION_TIME = 0.7    # 🔄 延長滑行反應時間
EMERGENCY_STOP_CM = 3.0  # ✨ 新增絕對安全距離

class YoloCmdListener(Node):
    def __init__(self):
        super().__init__('yolo_cmd_listener')
        # ✨ 新增狀態變量
        self.last_speed = 0.0
        self.safety_override = False
        self.last_log_time = 0

        self.cmd_discrete = "stop"
        self.angle_deg = None
        self.front_dist = float('inf')
        self.hcsr04_dist = float('inf')
        self.x_speed = 0.0
        self.last_logged_angle = None
        self.angle_ready = False
        self.has_stopped = False
        self.angle_time = 0.0
        self.cmd_time = 0.0
        self.ws_connected = False
        self.ws_warned = False

        # 訂閱器保持不變...
        self.create_subscription(String, '/yolo_cmd', self.cb_cmd, 10)
        self.create_subscription(Float32, '/yolo_angle', self.cb_angle, 10)
        self.create_subscription(LaserScan, '/filtered_scan', self.cb_scan, 10)
        self.create_subscription(Float32, '/ultrasonic_distance', self.cb_hcsr04, 10)
        self.create_subscription(Float32, '/x_speed', self.cb_xspeed, 10)
        self.create_subscription(Bool, '/websocket_connected', self.cb_ws, 1)

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.hcsr_stop_pub = self.create_publisher(Bool, '/hcsr04_enable', 1)

        self.create_timer(0.05, self.control_loop)  # 🔄 提高控制頻率 (20Hz)
        self.get_logger().info("✅ yolo_cmd_listener 啟動（安全優先模式）")

    # ... 保持其他回調函數不變 ...

    def control_loop(self):
        twist = Twist()
        now = time.time()

        # ✨ 1. 最高優先級：安全鎖定檢查
        if self.has_stopped:
            self.pub_vel.publish(twist)
            return

        # ✨ 2. 緊急停止條件（無視所有其他狀態）
        if self.hcsr04_dist < EMERGENCY_STOP_CM:
            self.stop_and_lock(f"🆘 緊急停止！距離 < {EMERGENCY_STOP_CM}cm")
            return

        # ✨ 3. 動態安全距離計算（考慮滑行）
        safe_dist = max(0.1, self.x_speed * REACTION_TIME + 0.05)  # 5cm緩衝
        if self.hcsr04_dist < safe_dist * 100:
            self.stop_and_lock(f"⚠️ 動態停止 | 安全距離: {safe_dist:.2f}m")
            return

        # ✨ 4. WebSocket 檢查
        if not self.ws_connected:
            if not self.ws_warned:
                self.get_logger().warn("🚩 WebSocket 斷線 → 停車")
                self.ws_warned = True
            self.pub_vel.publish(twist)
            return
        else:
            self.ws_warned = False

        # 5. 時效性檢查
        angle_valid = (now - self.angle_time) < EXPIRE_SEC
        cmd_valid = (now - self.cmd_time) < EXPIRE_SEC

        if not angle_valid and self.angle_deg is not None and abs(self.angle_deg) > 3.0:
            self.get_logger().warn("⏱️ 角度過期 → 清除")
            self.angle_deg = None

        # 6. YOLO 角度控制（次優先級）
        if self.angle_deg is not None and angle_valid:
            ang_err = math.radians(self.angle_deg)
            ang_abs = abs(ang_err)

            # ✨ 即使正在對準，也要即時檢查距離
            if self.hcsr04_dist < HC_STOP_CM:
                self.stop_and_lock(f"🛑 對準中但距離過近: {self.hcsr04_dist:.1f}cm")
                return

            lin_v = FORWARD_V * max(0.3, 1.0 - ang_abs * 2.0)
            ang_v = 0.0 if ang_abs < ANGLE_GATE else ANG_KP * ang_err

            # ✨ 加速度限制
            if abs(lin_v - self.last_speed) > 0.1:
                lin_v = self.last_speed + math.copysign(0.1, lin_v - self.last_speed)
            self.last_speed = lin_v

            twist.linear.x = lin_v
            twist.angular.z = ang_v

            # 日誌控制
            if now - self.last_log_time > 0.2:
                rounded_angle = round(self.angle_deg, 1)
                self.get_logger().info(
                    f"🎯 角度: {rounded_angle:+.1f}° | "
                    f"距離: {self.hcsr04_dist:.1f}cm | "
                    f"速度: {self.x_speed:.3f}m/s"
                )
                self.last_log_time = now

            self.pub_vel.publish(twist)
            return

        # 7. 非角度模式下的停止條件
        if self.hcsr04_dist < HC_STOP_CM:
            reason = (
                "無角度但曾對準" if self.angle_ready 
                else "無角度從未對準"
            )
            self.stop_and_lock(f"⚠️ {reason} + 距離 {self.hcsr04_dist:.1f}cm")
            return

        # 8. 基本指令處理
        if cmd_valid:
            if self.cmd_discrete == "left":
                twist.linear.x = FORWARD_V * 0.5
                twist.angular.z = 0.10
            elif self.cmd_discrete == "right":
                twist.linear.x = FORWARD_V * 0.5
                twist.angular.z = -0.10
            elif self.cmd_discrete in ["center", "forward"]:
                twist.linear.x = FORWARD_V

        self.pub_vel.publish(twist)

# ... 保持 main() 不變 ...