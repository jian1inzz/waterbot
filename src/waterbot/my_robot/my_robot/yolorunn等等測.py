#!/usr/bin/env python3
import math, time, rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ----------- 參數 ----------
FORWARD_V     = 0.05
ANG_KP        = 0.4
HC_STOP_CM    = 6.0  
EMERGENCY_STOP_CM = 3.5  # ✅ 加入緊急停止距離
ANGLE_GATE    = math.radians(3)
EXPIRE_SEC    = 4.0  # ❗ 指令與角度都 3 秒內有效
REACTION_TIME = 0.5  # ✅ 可調整的滑行反應延遲

class YoloCmdListener(Node):
    def __init__(self):
        super().__init__('yolo_cmd_listener')

        self.cmd_discrete = "stop"
        self.angle_deg = None
        self.front_dist = float('inf')
        self.hcsr04_dist = float('inf')
        self.x_speed = 0.0  # ✅ 新增編碼器速度
        self.last_logged_angle = None
        self.angle_ready = False
        self.has_stopped = False
        self.angle_time = 0.0
        self.cmd_time = 0.0
        self.ws_connected = False
        self.ws_warned = False

        self.create_subscription(String, '/yolo_cmd', self.cb_cmd, 10)
        self.create_subscription(Float32, '/yolo_angle', self.cb_angle, 10)
        self.create_subscription(LaserScan, '/filtered_scan', self.cb_scan, 10)
        self.create_subscription(Float32, '/ultrasonic_distance', self.cb_hcsr04, 10)
        self.create_subscription(Float32, '/x_speed', self.cb_xspeed, 10)  # ✅ 新增速度訂閱
        self.create_subscription(Bool, '/websocket_connected', self.cb_ws, 1)

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.hcsr_stop_pub = self.create_publisher(Bool, '/hcsr04_enable', 1)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("✅ yolo_cmd_listener 啟動（WebSocket 控制 + 自動停車 + 預測性滑行補償）")

    def cb_cmd(self, msg: String):
        self.cmd_discrete = msg.data.strip().lower()
        self.cmd_time = time.time()

    def cb_angle(self, msg: Float32):
        self.angle_deg = msg.data
        self.angle_time = time.time()

    def cb_scan(self, scan: LaserScan):
        inc = scan.angle_increment
        n15 = int(math.radians(15) / inc)
        self.front_dist = min(scan.ranges[:n15] + scan.ranges[-n15:])

    def cb_hcsr04(self, msg: Float32):
        if self.ws_connected and not self.has_stopped:
            self.hcsr04_dist = msg.data

    def cb_xspeed(self, msg: Float32):
        self.x_speed = msg.data  # ✅ 儲存來自編碼器的速度

    def cb_ws(self, msg: Bool):
        prev_state = self.ws_connected
        self.ws_connected = msg.data

        if not self.ws_connected and prev_state:
            self.get_logger().warn("🛘 WebSocket 已斷線！")
            self.ws_warned = False

    def stop_and_lock(self, reason: str):
        twist = Twist()
        self.pub_vel.publish(twist)
        self.hcsr_stop_pub.publish(Bool(data=False))
        self.get_logger().info(f"🚩 {reason} → 停車（鎖定）")
        self.has_stopped = True

    def control_loop(self):
        twist = Twist()

        # ✅ 停車鎖定狀態，維持停車
        if self.has_stopped:
            self.pub_vel.publish(twist)
            return

        # ✨ 2. 緊急停止條件（無視所有其他狀態）
        if self.hcsr04_dist < EMERGENCY_STOP_CM:
            self.stop_and_lock(f"🆘 緊急停止！距離 < {EMERGENCY_STOP_CM}cm")
            return

        if not self.ws_connected:
            if not self.ws_warned:
                self.get_logger().warn("🚩 WebSocket 尚未連線或已斷線 → 自動停車")
                self.ws_warned = True
            self.pub_vel.publish(twist)
            return
        else:
            self.ws_warned = False

        now = time.time()
        angle_valid = (now - self.angle_time) < EXPIRE_SEC
        cmd_valid = (now - self.cmd_time) < EXPIRE_SEC

        if not angle_valid and self.angle_deg is not None and abs(self.angle_deg) > 3.0:
            self.get_logger().warn("⏱️ 角度過期且大於 ±3° → 清除")
            self.angle_deg = None

        if not cmd_valid and self.cmd_discrete not in ["forward", "center"]:
            self.get_logger().warn(f"⏱️ 指令 {self.cmd_discrete} 已過期 → 停車")
            self.pub_vel.publish(twist)
            return

        # ✅ 預測性滑行補償判斷
        slip_dist = self.x_speed * REACTION_TIME
        if self.hcsr04_dist < slip_dist * 100:
            self.stop_and_lock(f"🚩 滑行補償：速度 {self.x_speed:.3f} m/s → 提前距離 {slip_dist:.2f} m")
            return

        if self.angle_deg is not None:
            ang_err = math.radians(self.angle_deg)
            ang_abs = abs(ang_err)

            if ang_abs < ANGLE_GATE:
                self.angle_ready = True
                if self.hcsr04_dist < HC_STOP_CM:
                    self.stop_and_lock(f"✅ 對準 + 距離 {self.hcsr04_dist:.1f} cm")
                    return

            lin_v = FORWARD_V * max(0.3, 1.0 - ang_abs * 2.0)
            ang_v = 0.0 if ang_abs < ANGLE_GATE else ANG_KP * ang_err

            twist.linear.x  = lin_v
            twist.angular.z = ang_v

            rounded_angle = round(self.angle_deg, 1)
            if self.last_logged_angle != rounded_angle:
                self.get_logger().info(f"🎯 角度: {rounded_angle:+.1f}° → 線速: {lin_v:.2f}, 角速: {ang_v:.2f}")
                self.last_logged_angle = rounded_angle

            self.pub_vel.publish(twist)
            return

        if self.angle_deg is None and self.angle_ready and self.hcsr04_dist < HC_STOP_CM:
            self.stop_and_lock(f"⚠️ 無角度但曾對準 + 距離 {self.hcsr04_dist:.1f} cm")
            return

        if self.angle_deg is None and not self.angle_ready and self.hcsr04_dist < HC_STOP_CM:
            self.stop_and_lock(f"⚠️ 無角度從未對準 + 距離 {self.hcsr04_dist:.1f} cm")
            return

        if cmd_valid:
            if self.cmd_discrete == "left":
                twist.linear.x  = FORWARD_V * 0.5
                twist.angular.z =  0.10
            elif self.cmd_discrete == "right":
                twist.linear.x  = FORWARD_V * 0.5
                twist.angular.z = -0.10
            elif self.cmd_discrete == "left_fast":
                twist.linear.x  = FORWARD_V * 0.5
                twist.angular.z =  0.08
            elif self.cmd_discrete == "right_fast":
                twist.linear.x  = FORWARD_V * 0.5
                twist.angular.z = -0.08
            elif self.cmd_discrete in ["center", "forward"]:
                twist.linear.x  = FORWARD_V

        self.pub_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = YoloCmdListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🚩 手動中斷")
    finally:
        node.pub_vel.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
