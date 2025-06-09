import RPi.GPIO as GPIO
import time

PUMP_RELAY_PIN = 17  # 根據你的接線選擇 GPIO 腳位

GPIO.setmode(GPIO.BCM)  # 使用 BCM 編號對應 GPIO17
GPIO.setup(PUMP_RELAY_PIN, GPIO.OUT, initial=GPIO.HIGH)  # 預設高電平，關閉繼電器（低電平觸發）

def pump_on():
    GPIO.output(PUMP_RELAY_PIN, GPIO.LOW)  # 低電平觸發，水泵啟動
    print("✅ 水泵已啟動")

def pump_off():
    GPIO.output(PUMP_RELAY_PIN, GPIO.HIGH)  # 高電平關閉繼電器，水泵停止
    print("🛑 水泵已停止")

def show_state():
    state = GPIO.input(PUMP_RELAY_PIN)
    level = "HIGH" if state == GPIO.HIGH else "LOW"
    print(f"🔍 目前 GPIO17 電位狀態為：{level}")

try:
    print("### 水泵繼電器控制程式啟動 ###")
    pump_off()  # 保持初始關閉狀態
    show_state()

    while True:
        user_input = input("\n按 Enter 切換水泵開關（輸入 q 離開）: ")

        if user_input.lower() == 'q':
            break

        current_state = GPIO.input(PUMP_RELAY_PIN)
        if current_state == GPIO.HIGH:
            pump_on()
        else:
            pump_off()

        show_state()

except KeyboardInterrupt:
    print("\n⚠️ 手動終止程式")

finally:
    pump_off()  # 保險停泵
    GPIO.cleanup()
    print("✅ GPIO 資源已清理，程式結束。")
