#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# 定義腳位
ENA_PIN = 17
IN1_PIN = 22
IN2_PIN = 27   # 加一個 IN2 腳位（你可以換成你用的腳位）

# 初始化 GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA_PIN, GPIO.OUT)
GPIO.setup(IN1_PIN, GPIO.OUT)
GPIO.setup(IN2_PIN, GPIO.OUT)

# 固定方向（IN1 高電位 = 正轉，IN2 低電位 = 不反轉）
GPIO.output(IN1_PIN, GPIO.HIGH)
GPIO.output(IN2_PIN, GPIO.LOW)  # ✅ 設為 LOW，避免浮接

print("✅ 水泵控制程式啟動")
try:
    while True:
        cmd = input("輸入 1 = 啟動水泵，0 = 停止，q = 離開：")

        if cmd == "1":
            GPIO.output(ENA_PIN, GPIO.HIGH)
            print("✅ 水泵已啟動")
        elif cmd == "0":
            GPIO.output(ENA_PIN, GPIO.LOW)
            print("🛑 水泵已停止")
        elif cmd.lower() == "q":
            break
        else:
            print("❓ 請輸入 1 / 0 / q")

except KeyboardInterrupt:
    print("⛔ 強制結束")

finally:
    GPIO.output(ENA_PIN, GPIO.LOW)
    GPIO.cleanup()
    print("🔚 GPIO 資源已清理")