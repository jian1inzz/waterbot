#!/usr/bin/env python3
import math, time, rclpy, random
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import os, signal, threading

# ----------- 參數 ----------
FORWARD_V     = 0.05
ANG_KP        = 0.13
HC_STOP_CM    = 7.0
PATROL_V     = 0.06
PATROL_ANG_V = 0.07  
EMERGENCY_STOP_CM = 3.5
ANGLE_GATE    = math.radians(3)
EXPIRE_SEC    = 3
REACTION_TIME = 2.05
SLIP_K        = 1.0   # ✅ 新增滑行修正係數

def monitor_parent():
    """🛡️ 當父進程被終止，這個子進程也會自動退出"""
    ppid = os.getppid()
    while True:
        if os.getppid() != ppid:
            print("🔴 父進程已死亡，終止 YOLO 指令節點")
            os.kill(os.getpid(), signal.SIGINT)
        time.sleep(1)

class YoloCmdListener(Node):
    def __init__(self):
        super().__init__('yolo_cmd_listener')

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
        self.cmd_expired_warned = False
        self.last_comp_log_time = 0.0
        self.last_yolo_time = 0.0 
        self.hcsr04_enabled = False

        self.create_subscription(Bool, '/pump_done', self.cb_pump_done, 10)
        self.create_subscription(String, '/yolo_cmd', self.cb_cmd, 10)
        self.create_subscription(Float32, '/yolo_angle', self.cb_angle, 10)
        self.create_subscription(Float32, '/obstacle_distance', self.cb_obstacle, 10)
        self.create_subscription(Float32, '/ultrasonic_distance', self.cb_hcsr04, 10)
        self.create_subscription(Float32, '/x_speed', self.cb_xspeed, 10)
        self.create_subscription(Bool, '/websocket_connected', self.cb_ws, 1)

        self.pub_vel = self.create_publisher(Twist, '/yolo_cmd_vel', 10)
        self.hcsr_stop_pub = self.create_publisher(Bool, '/hcsr04_enable', 1)

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("✅ yolo_cmd_listener 啟動（WebSocket 控制 + 自動停車 + 預測性滑行補償）")

    def cb_cmd(self, msg: String):
        self.cmd_discrete = msg.data.strip().lower()
        self.cmd_time = time.time()

    def cb_angle(self, msg: Float32):
        self.angle_deg = msg.data
        self.angle_time = time.time()
        self.last_yolo_time = self.angle_time

    def cb_obstacle(self, msg: Float32):
        self.front_dist = msg.data

    def cb_pump_done(self, msg: Bool):
        if msg.data:
            self.get_logger().info("💧 抽水完成 → 解鎖停車狀態（不主動開啟超音波）")
            self.has_stopped = False
            self.angle_ready = False
            self.hcsr04_dist = float('inf')
            self.hcsr04_enabled = False   # ✅ 重設旗標，允許重新啟用

        

    def cb_hcsr04(self, msg: Float32):
        # 只有在 WebSocket 已連線、且尚未停車時才處理
        if not (self.ws_connected and not self.has_stopped):
            return
        
        dist = msg.data
        
        # ❶ 讀值異常（過小）或 ❷ 感測器回傳 inf，都算「無效」
        if dist < 2.0 or math.isinf(dist):
            # 不寫入 self.hcsr04_dist，維持原本值 (通常是 inf)
            if dist < 2.0:
                self.get_logger().warn(f"⚠️ 超音波讀值異常：{dist:.2f} cm，已忽略")
            return
        
        # 讀值有效才更新
        self.hcsr04_dist = dist

    def cb_xspeed(self, msg: Float32):
        if not self.has_stopped:
            raw_speed = msg.data
            if self.hcsr04_dist < 15.0:
                raw_speed = min(max(raw_speed, 0.041), 0.48)
            self.x_speed = raw_speed

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

    def patrol_behavior(self):
        if self.has_stopped:
            return
        twist = Twist()
        twist.linear.x = PATROL_V
        self.pub_vel.publish(twist)


    def control_loop(self):

        twist = Twist()
        now = time.time()


        if self.has_stopped:
            self.pub_vel.publish(twist)
            self.get_logger().info("⚠️ 已停車狀態 → 不發送移動指令")
            return

        if self.hcsr04_dist < EMERGENCY_STOP_CM:
            self.stop_and_lock(f"🆘 緊急停止！距離 < {EMERGENCY_STOP_CM}cm")
            return

        if not self.ws_connected:
            if not self.ws_warned:
                self.get_logger().warn("🚩 WebSocket 尚未連線或已斷線 → 自動停車")
                self.ws_warned = True
            return
        else:
            self.ws_warned = False

        now = time.time()
        angle_valid = (now - self.angle_time) < EXPIRE_SEC
        cmd_valid = (now - self.cmd_time) < EXPIRE_SEC

        if not angle_valid and self.angle_deg is not None and abs(self.angle_deg) > 3.0:
            self.get_logger().warn("⏱️ 角度過期且大於 ±3° → 清除")
            self.angle_deg = None

        # ✅ 改為以角度為主的巡邏切換條件
        if not angle_valid:
            if not self.cmd_expired_warned:
                self.get_logger().warn("⏱️ 角度已過期 → 切換巡邏模式")
                self.cmd_expired_warned = True
            self.patrol_behavior()
            return
        else:
            self.cmd_expired_warned = False

        should_enable = (
            self.front_dist < 0.35 and 
            (now - self.last_yolo_time) < 5.0 and 
            self.angle_deg is not None and 
            abs(self.angle_deg) < 5.0
        )

        

        if should_enable and not self.hcsr04_enabled:
            self.hcsr_stop_pub.publish(Bool(data=True))
            self.hcsr04_enabled = True
            self.get_logger().info("✅ 啟用超音波")
        
        if self.hcsr04_enabled and (
            self.angle_deg is None or abs(self.angle_deg) > 60.0
        ):
            self.hcsr_stop_pub.publish(Bool(data=False))
            self.hcsr04_enabled = False
            self.get_logger().info(f"❌ 角度偏移 {self.angle_deg:.1f}° → 關閉超音波")

        slip_dist = self.x_speed * REACTION_TIME * SLIP_K
        predicted_stop_cm = slip_dist * 100

        # ➊ 只有 hcsr04_dist 不是 inf 才印 log
        if now - self.last_comp_log_time >= 1.0:
            if not math.isinf(self.hcsr04_dist):
                self.get_logger().info(
                    f"🧪 [補償判斷] hcsr04={self.hcsr04_dist:.2f} cm, "
                    f"預測滑行距離={predicted_stop_cm:.2f} cm"
                )
            # 想完全靜默可把 else 刪掉
            self.last_comp_log_time = now


        # ➋ 只有有效距離才比較
        if (not math.isinf(self.hcsr04_dist)) and self.hcsr04_dist < predicted_stop_cm:
            self.get_logger().info(
                f"🧪 [補償觸發] hcsr04={self.hcsr04_dist:.2f} cm, "
                f"預測滑行距離={predicted_stop_cm:.2f} cm"
            )
            self.stop_and_lock(
                f"🚩 滑行補償：速度 {self.x_speed:.3f} m/s → 預測 {predicted_stop_cm:.2f} cm"
            )
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
    threading.Thread(target=monitor_parent, daemon=True).start()
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