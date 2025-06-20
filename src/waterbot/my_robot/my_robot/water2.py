#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
import threading
from std_msgs.msg import Bool 
import os
import signal
import threading

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


def monitor_parent():
    """🛡️ 當父進程被終止，這個子進程也會自動退出"""
    ppid = os.getppid()
    while True:
        if os.getppid() != ppid:
            print("🔴 父進程已死亡，終止抽水節點")
            os.kill(os.getpid(), signal.SIGINT)
        time.sleep(1)

class RailPumpController(Node):
    def __init__(self):
        super().__init__('rail_pump_controller_node')
        self.current_position_mm = 0  # ✅ 起始位置改為 0 mm
        self.step_delay = 0.0003
        self.pause_flag = threading.Event()
        self.pause_flag.set()
        self.create_subscription(Bool, '/water_pump', self.cb_water, 10)
        self.flow_running = False
        self.movement_lock = threading.Lock()  # 避免多執行緒錯亂
        self.setup_gpio()
        self.moved_mm_this_round = 0.0
        self.last_water_state = False
        

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
    
    def cb_water(self, msg: Bool):
        if msg.data == self.last_water_state:
            return  # 狀態沒有變化，直接跳過

        self.last_water_state = msg.data  # 更新狀態

        if msg.data:
            if self.flow_running:
                self.get_logger().warn("⚠️ 已在執行流程，忽略重複啟動")
                return
            self.get_logger().info("💧 收到抽水指令 ON，啟動流程")
            self.flow_running = True
            self.pause_flag.set()
            self.moved_mm_this_round = 0.0
            threading.Thread(target=self.auto_flow, daemon=True).start()
        else:
            if not self.flow_running:
                self.get_logger().warn("⚠️ 流程尚未啟動，忽略關閉指令")
                return
            self.get_logger().info("🛑 收到抽水指令 OFF，停止水泵＋導軌退回")
            self.flow_running = False
            self.stop_pump()

            back_mm = self.current_position_mm
            if back_mm > 0:
                def rollback():
                    self.get_logger().info("🔁 GPIO BACK 啟動中...")
                    time.sleep(1.0)
                    self.get_logger().info(f"🔙 中斷流程，導軌退回 {back_mm:.2f} mm")
                    self.pause_flag.set()
                    self.move_mm(back_mm, direction="backward")
                    self.current_position_mm = 0.0

                threading.Thread(target=rollback, daemon=True).start()




    def move_mm(self, mm, direction="forward"):
        self.pause_flag.set()
        time.sleep(0.002)
        total_steps = int(mm * steps_per_mm)
        actual_steps_done = 0

        # ✅ 提前中斷點（不能卡住 movement_lock）
        if direction == "forward" and not self.flow_running:
            self.get_logger().warn("⛔ 中斷：尚未進入 lock，略過移動")
            return

        # ✅ 進入 lock，保證安全移動
        with self.movement_lock:
            # 第二層保險（若中斷剛好發生在進 lock 前後）
            if direction == "forward" and not self.flow_running:
                self.get_logger().warn("⛔ 中斷：進入 lock 後偵測到 flow 已停止，跳出")
                return

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
                time.sleep(0.001)
            else:
                self.get_logger().error("❌ 錯誤方向")
                return

            self.get_logger().info(f"⚙️ 移動 {direction} {mm:.3f} mm（共 {total_steps} 步）")

            for step in range(total_steps):
                self.pause_flag.wait()

                # ✅ 中途中斷檢查（for 迴圈內）
                if direction == "forward" and not self.flow_running:
                    self.get_logger().warn(f"⛔ 中途中斷於第 {step}/{total_steps} 步，提早停止")
                    break

                GPIO.output(STEP_PIN, GPIO.HIGH)
                time.sleep(self.step_delay)
                GPIO.output(STEP_PIN, GPIO.LOW)
                time.sleep(self.step_delay)
                actual_steps_done += 1

            actual_mm = actual_steps_done / steps_per_mm

            if direction == "forward":
                self.current_position_mm += actual_mm
                self.moved_mm_this_round += actual_mm
            else:
                self.current_position_mm -= actual_mm

            self.get_logger().info(f"✅ 移動完成：{actual_mm:.2f} mm，當前位置 {self.current_position_mm:.2f} mm")







    def start_pump(self):
        self.get_logger().info("🚿 啟動水泵")
        GPIO.output(PUMP_ENA, GPIO.HIGH)

    def stop_pump(self):
        GPIO.output(PUMP_ENA, GPIO.LOW)
        self.get_logger().info("🛑 停止水泵")

    def auto_flow(self):
        self.get_logger().info("🚀 啟動導軌＋水泵聯動流程")

        self.move_mm(200, direction="forward")
        if not self.flow_running:
            self.get_logger().info(f"🔙 中斷流程，導軌退回 {self.current_position_mm:.2f} mm")
            self.move_mm(self.current_position_mm, direction="backward")
            self.current_position_mm = 0.0
            return

        self.get_logger().info("⌛ 等待 2 秒準備抽水")
        time.sleep(2)
        if not self.flow_running:
            self.get_logger().info(f"🔙 中斷流程，導軌退回 {self.current_position_mm:.2f} mm")
            self.move_mm(self.current_position_mm, direction="backward")
            self.current_position_mm = 0.0
            return

        self.start_pump()
        time.sleep(13)
        if not self.flow_running:
            self.get_logger().info("🛑 中斷抽水中")
            self.stop_pump()
            self.get_logger().info(f"🔙 中斷流程，導軌退回 {self.current_position_mm:.2f} mm")
            self.move_mm(self.current_position_mm, direction="backward")
            self.current_position_mm = 0.0
            return

        self.stop_pump()
        self.get_logger().info("⏳ 抽水結束，等待 2 秒再回退")
        time.sleep(2)
        if not self.flow_running:
            self.get_logger().info(f"🔙 中斷流程，導軌退回 {self.current_position_mm:.2f} mm")
            self.move_mm(self.current_position_mm, direction="backward")
            self.current_position_mm = 0.0
            return

        self.move_mm(200, direction="backward")
        self.current_position_mm = 0.0
        self.get_logger().info("🎉 聯動流程完成！")

def main(args=None):
    threading.Thread(target=monitor_parent, daemon=True).start()
    rclpy.init(args=args)
    node = RailPumpController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        GPIO.cleanup()  # ← 安全釋放 GPIO
        rclpy.shutdown()

if __name__ == '__main__':
    main()
