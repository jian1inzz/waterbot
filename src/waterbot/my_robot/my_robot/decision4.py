#!/usr/bin/env python3
import math, rclpy, random
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import os, signal, threading


FORWARD_V     = 0.07
TURN_V        = 0.08
BACKWARD_V    = -0.05

TURN_TIME_BASE     = 20
BACKWARD_TIME      = 14
POST_TURN_DELAY_STEP = 8

SAFE_FRONT = 0.38
LEFT_FRONT_SAFE  = 0.28
RIGHT_FRONT_SAFE = 0.28

def monitor_parent():
    """🛡️ 當父進程被終止，這個子進程也會自動退出"""
    ppid = os.getppid()
    while True:
        if os.getppid() != ppid:
            print("🔴 父進程已死亡，終止 YOLO 指令節點")
            os.kill(os.getpid(), signal.SIGINT)
        time.sleep(1)

class ObstacleAvoidNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoid_node')

        self.pub_cmdvel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/yolo_cmd_vel', self.cb_yolo_cmd, 10)
        self.create_subscription(Float32, '/yolo_angle', self.cb_yolo_angle, 10)  # ✅ 新增角度訂閱

        self.create_subscription(Float32, '/obstacle_distance', self.cb_obstacle, 10)
        self.create_subscription(Float32, '/obstacle_left', self.cb_obstacle_left, 10)
        self.create_subscription(Float32, '/obstacle_right', self.cb_obstacle_right, 10)
        self.create_subscription(Float32, '/obstacle_left_front', self.cb_left_front, 10)
        self.create_subscription(Float32, '/obstacle_right_front', self.cb_right_front, 10)
        self.create_subscription(Bool, '/hcsr04_enable', self.cb_hcsr04, 10)

        self.create_timer(0.3, self.avoid_loop)

        self.state = 'straight'
        self.counter = 0
        self.last_turn_direction = 1
        self.post_turn_delay_cnt = 0
        self.realign_counter = 0

        self.obstacle_dist = float('inf')
        self.left_dist = float('inf')
        self.right_dist = float('inf')
        self.left_front_dist = float('inf')
        self.right_front_dist = float('inf')
        self.last_logged_cmd = None

        self.latest_cmd = Twist()

        # ✅ YOLO 角度判斷用變數
        self.angle_deg = None
        self.last_yolo_time = 0.0
        self.yolo_locked = False

        self.hcsr04_enabled = False
        self.cooldown_until = 0

        self.get_logger().info("✅ Obstacle Avoid Node（避障 + 指令轉發）啟動")

    def _round(self, x, digits=2):
        return round(x, digits)

    def cb_yolo_cmd(self, msg: Twist):
        if self.state != 'straight':
            return  # ❌ 目前正在避障，不接受 yolo_cmd_vel
        self.latest_cmd = msg
        rounded = (self._round(msg.linear.x), self._round(msg.angular.z))
        if rounded != self.last_logged_cmd:
            self.get_logger().info(
                f"[收到 yolo_cmd_vel] linear.x={rounded[0]:.2f}, "
                f"angular.z={rounded[1]:.2f}"
            )
            self.last_logged_cmd = rounded

    def cb_yolo_angle(self, msg: Float32):  # ✅ 新增角度判斷
        self.angle_deg = msg.data
        self.last_yolo_time = time.time()
        self.yolo_locked = abs(self.angle_deg) < 10.0

    def cb_hcsr04(self, msg: Bool):
        self.get_logger().info(f"📩 一開始收到 /hcsr04_enable = {msg.data}")
        if self.hcsr04_enabled and (not msg.data):
            now = self.get_clock().now().seconds_nanoseconds()[0]
            self.cooldown_until = now + 10
            self.get_logger().info("🕒 停車完成 → 10 秒冷卻再恢復避障")
        self.hcsr04_enabled = msg.data

    def cb_obstacle(self, msg: Float32):
        self.obstacle_dist = msg.data

    def cb_obstacle_left(self, msg: Float32):
        self.left_dist = msg.data

    def cb_obstacle_right(self, msg: Float32):
        self.right_dist = msg.data

    def cb_left_front(self, msg: Float32):
        self.left_front_dist = msg.data

    def cb_right_front(self, msg: Float32):
        self.right_front_dist = msg.data

    def send_stop(self):
        self.pub_cmdvel.publish(Twist())

    def is_narrow_pass(self):
        return self.left_dist < 0.3 and self.right_dist < 0.3 and self.obstacle_dist > 0.3

    def avoid_loop(self):
        twist = Twist()
        now = self.get_clock().now().seconds_nanoseconds()[0]

        if now < self.cooldown_until:
            self.send_stop()
            return

        # ✅ 根據是否為 YOLO 鎖定目標，調整避障靈敏度
        recent_yolo = (time.time() - self.last_yolo_time) < 3.0
        if recent_yolo and self.yolo_locked:
            dynamic_safe_front = 0.35
        else:
            dynamic_safe_front = SAFE_FRONT

        if self.state == 'straight':
            # ✅ 絕對優先處理避障：正前 + 左前 + 右前任一太近就觸發
            if self.hcsr04_enabled:
                danger_front = False
                self.get_logger().info("🔕 超音波啟用，略過前方雷達避障")
            else:
                danger_front = self.obstacle_dist < dynamic_safe_front
            danger_left  = self.left_front_dist < LEFT_FRONT_SAFE
            danger_right = self.right_front_dist < RIGHT_FRONT_SAFE

            # ✅ YOLO 指令向前，但實際上前方有障礙 → 視為觸發避障
            yolo_conflict = (
                self.latest_cmd.linear.x > 0 and 
                (danger_front or danger_left or danger_right)
            )

            if (danger_front or danger_left or danger_right or yolo_conflict) and not self.is_narrow_pass():
                self.state = 'backward'
                self.counter = 0
                twist.linear.x = BACKWARD_V
                self.pub_cmdvel.publish(twist)
                self.get_logger().warn(
                    f"[backward] ⛔ 偵測障礙物或 YOLO 指令衝突 → 後退！"
                    f"前={self.obstacle_dist:.2f}, 左前={self.left_front_dist:.2f}, 右前={self.right_front_dist:.2f}"
                )
                return

            # ✅ 發送 YOLO 指令（安全情況下）
            self.pub_cmdvel.publish(self.latest_cmd)
            return



        elif self.state == 'backward':
            twist.linear.x = BACKWARD_V
            self.counter += 1
            if self.counter >= BACKWARD_TIME:
                self.send_stop()
                self.state = 'turn'
                self.counter = 0
                if self.angle_deg is not None and abs(self.angle_deg) > 3:
                    self.turn_direction = -1 if self.angle_deg < 0 else 1
                    self.get_logger().info(f"🎯 YOLO角度 {self.angle_deg:.1f}° → 優先轉向 {'左' if self.turn_direction == -1 else '右'}")
                else:
                    left_space  = min(self.left_dist, self.left_front_dist)
                    right_space = min(self.right_dist, self.right_front_dist)

                    if left_space > right_space:
                        self.turn_direction = 1
                    else:
                        self.turn_direction = -1

                    if self.turn_direction == 1 and self.left_dist < 0.3:
                        self.turn_direction = -1
                        self.get_logger().info("↪️ 左側過近 → 改右轉")
                    elif self.turn_direction == -1 and self.right_dist < 0.3:
                        self.turn_direction = 1
                        self.get_logger().info("↩️ 右側過近 → 改左轉")

                self.last_turn_direction = self.turn_direction
                self.turn_duration = random.randint(10, TURN_TIME_BASE)
                self.get_logger().info(f"[turn] 後退後轉向 {self.turn_direction}（靠近牆策略）")
                return

        elif self.state == 'turn':
            twist.linear.x = FORWARD_V * 0.5

            if self.angle_deg is not None and abs(self.angle_deg) > 3 and (time.time() - self.last_yolo_time) < 3.0:
                twist.angular.z = self.turn_direction * TURN_V * 0.25
            else:
                twist.angular.z = self.turn_direction * TURN_V * 0.5

            self.counter += 1
            if self.counter >= self.turn_duration:
                self.send_stop()
                self.state = 'pause'
                self.post_turn_delay_cnt = 0
                self.get_logger().info("[pause] 轉彎完畢，暫停 {:.1f}s 再前進".format(POST_TURN_DELAY_STEP * 0.3))
                return

        elif self.state == 'pause':
            self.post_turn_delay_cnt += 1
            if self.post_turn_delay_cnt >= POST_TURN_DELAY_STEP:
                self.state = 'realign'
                self.realign_counter = 0
                self.get_logger().info("[realign] 進行回正")
            self.send_stop()
            return

        elif self.state == 'realign':
            if self.left_dist < 0.25 and self.right_dist < 0.25:
                self.get_logger().info("⚠️ 窄通道內跳過導正")
                self.state = 'straight'
                self.counter = 0
                return

            twist.angular.z = -self.last_turn_direction * TURN_V * 0.5
            self.realign_counter += 1
            if self.realign_counter >= 4:
                self.send_stop()
                self.state = 'straight'
                self.counter = 0
                self.get_logger().info("[straight] 回正結束 → 繼續巡邏")
                return

        self.pub_cmdvel.publish(twist)

def main(args=None):
    threading.Thread(target=monitor_parent, daemon=True).start()
    rclpy.init(args=args)
    node = ObstacleAvoidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import time  # ✅ 放這裡避免全域影響
    main()
