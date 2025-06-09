#!/usr/bin/env python3
# yolo_cmd_listener_v2_hcsr_stop_lock.py ── 小於 6 cm 停車，停車後鎖定並關閉 HC-SR04 發布（含三種停車情境）

import math, rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ----------- 參數 ----------
FORWARD_V   = 0.05
ANG_KP      = 0.4
HC_STOP_CM  = 6.0
ANGLE_GATE  = math.radians(3)

class YoloCmdListener(Node):
    def __init__(self):
        super().__init__('yolo_cmd_listener')

        self.cmd_discrete = "stop"
        self.angle_deg    = None
        self.front_dist   = float('inf')
        self.hcsr04_dist  = float('inf')
        self.last_logged_angle = None
        self.angle_ready = False
        self.has_stopped = False

        self.create_subscription(String,   '/yolo_cmd',            self.cb_cmd,   10)
        self.create_subscription(Float32,  '/yolo_angle',          self.cb_angle, 10)
        self.create_subscription(LaserScan,'/filtered_scan',       self.cb_scan,  10)
        self.create_subscription(Float32,  '/ultrasonic_distance', self.cb_hcsr04, 10)

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.hcsr_stop_pub = self.create_publisher(Bool, '/hcsr04_enable', 1)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("✅ yolo_cmd_listener v2 啟動（三種 6cm 停車邏輯）")

    def cb_cmd(self, msg: String):
        self.cmd_discrete = msg.data.strip().lower()

    def cb_angle(self, msg: Float32):
        self.angle_deg = msg.data

    def cb_scan(self, scan: LaserScan):
        inc = scan.angle_increment
        n15 = int(math.radians(15) / inc)
        self.front_dist = min(scan.ranges[:n15] + scan.ranges[-n15:])

    def cb_hcsr04(self, msg: Float32):
        if not self.has_stopped:
            self.hcsr04_dist = msg.data

    def stop_and_lock(self, reason: str):
        twist = Twist()
        self.pub_vel.publish(twist)
        self.hcsr_stop_pub.publish(Bool(data=False))
        self.get_logger().info(f"🛑 {reason} → 停車（鎖定）")
        self.has_stopped = True

    def control_loop(self):
        twist = Twist()

        if self.has_stopped:
            self.pub_vel.publish(twist)
            return

        # --- 第一優先：對準 + 6cm
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

        # --- 第二優先：曾對準過 + 6cm
        if self.angle_deg is None and self.angle_ready and self.hcsr04_dist < HC_STOP_CM:
            self.stop_and_lock(f"⚠️ 無角度但曾對準 + 距離 {self.hcsr04_dist:.1f} cm")
            return

        # --- 第三優先：沒角度從未對準過 + 6cm
        if self.angle_deg is None and not self.angle_ready and self.hcsr04_dist < HC_STOP_CM:
            self.stop_and_lock(f"⚠️ 無角度從未對準 + 距離 {self.hcsr04_dist:.1f} cm")
            return

        # 離散指令控制
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
        node.get_logger().info("🛑 手動中斷")
    finally:
        node.pub_vel.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
