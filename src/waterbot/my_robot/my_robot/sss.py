#!/usr/bin/env python3
import math, time, rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ----------- 參數 ----------
FORWARD_V     = 0.05
ANG_KP        = 0.07
HC_STOP_CM    = 6.3
EMERGENCY_STOP_CM = 3.5
ANGLE_GATE    = math.radians(3)
EXPIRE_SEC    = 5
REACTION_TIME = 2.05
SLIP_K        = 1.0

class YoloCmdListener(Node):
    def __init__(self):
        super().__init__('yolo_cmd_listener')

        # ----- 狀態 -----
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

        self.yolo_locked = False
        self.locked_cup_position = None
        self.last_yolo_seen_time = 0.0
        self.yolo_timeout = 5.0
        self.radar_triggered_once = False

        self.allow_yolo_detect = True
        self.post_stop_phase = None
        self.post_stop_time = 0.0
        self.boot_time = time.time()
        self.last_turn_time = 0.0 

        self.is_turning = False  # ✅ 轉彎狀態

        # ----- 訂閱 -----
        self.create_subscription(String, '/yolo_cmd', self.cb_cmd, 10)
        self.create_subscription(Float32, '/yolo_angle', self.cb_angle, 10)
        self.create_subscription(Float32, '/ultrasonic_distance', self.cb_hcsr04, 10)
        self.create_subscription(Float32, '/x_speed', self.cb_xspeed, 10)
        self.create_subscription(Bool, '/websocket_connected', self.cb_ws, 1)
        self.create_subscription(Float32, '/obstacle_distance', self.cb_front_dist, 10)
        self.create_subscription(Bool, '/is_turning', self.cb_turning, 10)  # ✅ 訂閱轉彎狀態

        # ----- 發布 -----
        self.pub_suggest = self.create_publisher(Twist, '/yolo_cmd_vel', 10)
        self.hcsr_stop_pub = self.create_publisher(Bool, '/hcsr04_enable', 1)
        self.pub_yolo_done = self.create_publisher(Bool, '/yolo_done', 1)

        # ----- 主迴圈 -----
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("✅ yolo_cmd_listener 啟動（WebSocket 控制 + 自動停車 + 預測性滑行補償）")

    def cb_cmd(self, msg: String):
        self.cmd_discrete = msg.data.strip().lower()
        self.cmd_time = time.time()

    def cb_angle(self, msg: Float32):
        if not self.allow_yolo_detect or self.is_turning:  # ✅ 轉彎期間不收角度
            return
        self.angle_deg = msg.data
        self.angle_time = time.time()
        self.yolo_locked = True
        self.last_yolo_seen_time = time.time()
        self.locked_cup_position = self.angle_deg

    def cb_turning(self, msg: Bool):
        self.is_turning = msg.data
        if not msg.data:
            self.last_turn_time = time.time()  # 轉彎結束時間

    def cb_hcsr04(self, msg: Float32):
        if self.ws_connected and not self.has_stopped:
            if msg.data < 2.0:
                self.get_logger().warn(f"⚠️ 超音波讀值異常：{msg.data:.2f} cm，已忽略")
                return
            self.hcsr04_dist = msg.data

    def cb_front_dist(self, msg: Float32):
        self.front_dist = msg.data
        if self.front_dist < 0.4:
            self.radar_triggered_once = True

    def cb_xspeed(self, msg: Float32):
        if not self.has_stopped:
            raw_speed = msg.data
            if self.hcsr04_dist < 15.0:
                raw_speed = max(0.041, min(raw_speed, 0.48))
            self.x_speed = raw_speed

    def cb_ws(self, msg: Bool):
        prev_state = self.ws_connected
        self.ws_connected = msg.data
        if not self.ws_connected and prev_state:
            self.get_logger().warn("🛘 WebSocket 已斷線！")
            self.ws_warned = False

    def stop_and_lock(self, reason: str):
        twist = Twist()
        self.pub_suggest.publish(twist)
        self.hcsr_stop_pub.publish(Bool(data=False))
        self.get_logger().info(f"🚩 {reason} → 停車（鎖定）")
        self.has_stopped = True
        self.post_stop_phase = "wait_stop"
        self.post_stop_time = time.time()

    def control_loop(self):
        now = time.time()
        twist = Twist()

        if now - self.boot_time < 8.0:
            if not hasattr(self, 'boot_warn_logged'):
                self.get_logger().warn("⏳ 啟動初期（前 10 秒），暫不送出指令")
                self.boot_warn_logged = True
            return

        if self.is_turning:
            self.pub_suggest.publish(Twist())
            return  # ✅ 轉彎中不發送控制指令

        if self.post_stop_phase is not None:
            elapsed = now - self.post_stop_time

            if self.post_stop_phase == "wait_stop":
                if elapsed >= 5.0:
                    self.post_stop_phase = "backward"
                    self.post_stop_time = now
                    self.get_logger().info("⏳ 停車等待完成 → 進入後退階段")
                self.pub_suggest.publish(Twist())
                return

            elif self.post_stop_phase == "backward":
                if elapsed < 1.0:
                    twist.linear.x = -0.05
                    self.pub_suggest.publish(twist)
                    return
                else:
                    self.post_stop_phase = "wait_enable_yolo"
                    self.post_stop_time = now
                    self.allow_yolo_detect = False
                    self.pub_yolo_done.publish(Bool(data=True))
                    self.get_logger().info("🔁 切換巡邏模式，暫停 YOLO 偵測 2 秒")

                    # ✅ 主動清掉記憶，防止兩秒後又跳回 YOLO 模式
                    self.yolo_locked = False
                    self.angle_ready = False
                    self.angle_deg = None
                    self.locked_cup_position = None
                    self.cmd_discrete = "stop"
                    return


            elif self.post_stop_phase == "wait_enable_yolo":
                if elapsed >= 2.0:
                    self.allow_yolo_detect = True
                    self.post_stop_phase = None
                    self.get_logger().info("🟢 恢復 YOLO 偵測")
                self.pub_suggest.publish(Twist())
                return

        if time.time() - self.last_yolo_seen_time > self.yolo_timeout:
            self.yolo_locked = False

        if self.has_stopped:
            self.pub_suggest.publish(twist)
            return

        if self.yolo_locked and self.hcsr04_dist < EMERGENCY_STOP_CM:
            self.stop_and_lock(f"🆘 緊急停止！距離 < {EMERGENCY_STOP_CM}cm")
            return

        if not self.ws_connected:
            if not self.ws_warned:
                self.get_logger().warn("⚠️ WebSocket 尚未連線（但不強制停車）")
                self.ws_warned = True
            # ✅ 這裡不要 return，不送 0 速，讓後面繼續跑
        else:
            self.ws_warned = False

        angle_valid = (now - self.angle_time) < EXPIRE_SEC
        cmd_valid = (now - self.cmd_time) < EXPIRE_SEC

        if not angle_valid and self.angle_deg is not None and abs(self.angle_deg) > 3.0:
            self.get_logger().warn("⏱️ 角度過期且大於 ±3° → 清除")
            self.angle_deg = None

        #if not cmd_valid and self.cmd_discrete not in ["forward", "center"]:
            #if not self.cmd_expired_warned:
        #        self.get_logger().warn(f"⏱️ 指令 {self.cmd_discrete} 已過期 → 停車")
        #        self.cmd_expired_warned = True
        #    self.pub_suggest.publish(twist)
        #    return
        #else:
        #    self.cmd_expired_warned = False

        slip_dist = self.x_speed * REACTION_TIME * SLIP_K
        predicted_stop_cm = slip_dist * 100

        if now - self.last_comp_log_time >= 1.0 and not math.isinf(self.hcsr04_dist):
            self.get_logger().info(
                f"🧪 [補償判斷] hcsr04={self.hcsr04_dist:.2f} cm, 預測滑行距離={predicted_stop_cm:.2f} cm"
            )
            self.last_comp_log_time = now

        # ---- 原本的補償觸發改成忽略 inf ----
        angle_condition_ok = (
            self.locked_cup_position is not None and abs(self.locked_cup_position) < 8.0
        )

        if (
            self.yolo_locked
            and not math.isinf(self.hcsr04_dist)
            and self.hcsr04_dist < predicted_stop_cm
            and (now - self.last_turn_time > 1.5)
        ):


            self.get_logger().info(
                f"🧪 [補償觸發] hcsr04={self.hcsr04_dist:.2f} cm, 預測滑行距離={predicted_stop_cm:.2f} cm"
            )
            self.stop_and_lock(
                f"🚩 滑行補償：速度 {self.x_speed:.3f} m/s → 預測 {predicted_stop_cm:.2f} cm"
            )
            return

        if self.yolo_locked and self.locked_cup_position is not None:
            if self.radar_triggered_once:
                self.hcsr_stop_pub.publish(Bool(data=True))

            ang_err = math.radians(self.locked_cup_position)
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

            rounded_angle = round(self.locked_cup_position, 1)
            if self.last_logged_angle != rounded_angle:
                self.get_logger().info(f"🎯 角度: {rounded_angle:+.1f}° → 線速: {lin_v:.2f}, 角速: {ang_v:.2f}")
                self.last_logged_angle = rounded_angle

            self.pub_suggest.publish(twist)
            return

        if self.angle_deg is None and self.angle_ready and self.yolo_locked and self.hcsr04_dist < HC_STOP_CM:
            self.stop_and_lock(f"⚠️ 無角度但曾對準 + 距離 {self.hcsr04_dist:.1f} cm")
            return

        if self.angle_deg is None and not self.angle_ready and self.yolo_locked and self.hcsr04_dist < HC_STOP_CM:
            self.stop_and_lock(f"⚠️ 無角度從未對準 + 距離 {self.hcsr04_dist:.1f} cm")
            return

def main(args=None):
    rclpy.init(args=args)
    node = YoloCmdListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🚩 手動中斷")
    finally:
        node.pub_suggest.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
