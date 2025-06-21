#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
import threading
from std_msgs.msg import Bool 

# ---------------- GPIO 腳位定義 ----------------
DIR_PIN = 20
STEP_PIN = 21
EN_PIN = 16
PUMP_ENA = 17
PUMP_IN1 = 22
PUMP_IN2 = 27

# ---------------- 馬達參數 ----------------
STEPS_PER_REV = 200
MICROSTEP = 8
LEAD_MM_PER_REV = 8
MAX_POSITION_MM = 200
MIN_POSITION_MM = 0
steps_per_mm = (STEPS_PER_REV * MICROSTEP) / LEAD_MM_PER_REV

class RailPumpController(Node):
    def __init__(self):
        super().__init__('rail_pump_controller_node')
        self.current_position_mm = 0  # ✅ 起始位置改為 0 mm
        self.step_delay = 0.0003
        self.pause_flag = threading.Event()
        self.pause_flag.set()
        self.pub_pump_done = self.create_publisher(Bool, '/pump_done', 10)
        self.create_subscription(Bool, '/start_pump', self.cb_start_pump, 10)


        self.setup_gpio()

    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(DIR_PIN, GPIO.OUT)
        GPIO.setup(STEP_PIN, GPIO.OUT)
        GPIO.setup(EN_PIN, GPIO.OUT)
        GPIO.output(EN_PIN, GPIO.LOW)

        GPIO.setup(PUMP_ENA, GPIO.OUT)
        GPIO.setup(PUMP_IN1, GPIO.OUT)
        GPIO.setup(PUMP_IN2, GPIO.OUT)
        GPIO.output(PUMP_IN1, GPIO.HIGH)
        GPIO.output(PUMP_IN2, GPIO.LOW)
        GPIO.output(PUMP_ENA, GPIO.LOW)
        self.get_logger().info("✅ GPIO 初始化完成")

    def cb_start_pump(self, msg: Bool):
        if msg.data:
            self.get_logger().info("📩 收到 /start_pump 啟動抽水指令")
            threading.Thread(target=self.auto_flow, daemon=True).start()

    def move_mm(self, mm, direction="forward"):
        total_steps = int(mm * steps_per_mm)
        if direction == "forward":
            if self.current_position_mm + mm > MAX_POSITION_MM:
                self.get_logger().warn("❌ 超出最大位置")
                return
            GPIO.output(DIR_PIN, GPIO.HIGH)
        elif direction == "backward":
            if self.current_position_mm - mm < MIN_POSITION_MM:
                self.get_logger().warn("❌ 超出最小位置")
                return
            GPIO.output(DIR_PIN, GPIO.LOW)
        else:
            self.get_logger().error("❌ 錯誤方向")
            return

        self.get_logger().info(f"⚙️ 移動 {direction} {mm} mm（共 {total_steps} 步）")
        for _ in range(total_steps):
            self.pause_flag.wait()
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(self.step_delay)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(self.step_delay)

        self.current_position_mm += mm if direction == "forward" else -mm
        self.get_logger().info(f"✅ 目前位置：{self.current_position_mm:.2f} mm")

    def start_pump(self):
        self.get_logger().info("🚿 啟動水泵")
        GPIO.output(PUMP_ENA, GPIO.HIGH)

    def stop_pump(self):
        GPIO.output(PUMP_ENA, GPIO.LOW)
        self.get_logger().info("🛑 停止水泵")

    def auto_flow(self):
        self.get_logger().info("🚀 啟動導軌＋水泵聯動流程")
        self.move_mm(200, direction="forward")
        self.get_logger().info("⌛ 等待 2 秒準備抽水")
        time.sleep(2)
        self.start_pump()
        time.sleep(13)
        self.stop_pump()
        self.get_logger().info("⏳ 抽水結束，等待 2 秒再回退")
        time.sleep(2)
        self.move_mm(200, direction="backward")
        self.get_logger().info("🎉 聯動流程完成！")

        # ✅ 發布抽水完成訊號
        self.pub_pump_done.publish(Bool(data=True))
        self.get_logger().info("✅ 已發布 /pump_done = True")

    def destroy_node(self):
        self.stop_pump()
        GPIO.output(EN_PIN, GPIO.HIGH)
        GPIO.cleanup()
        self.get_logger().info("🔚 清理 GPIO 資源")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RailPumpController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
