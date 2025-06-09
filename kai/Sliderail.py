import RPi.GPIO as GPIO
import time
import threading

# GPIO 腳位設定
DIR_PIN = 20
STEP_PIN = 21
EN_PIN = 16

# 馬達參數
STEPS_PER_REV = 200
MICROSTEP = 8
LEAD_MM_PER_REV = 8
MAX_POSITION_MM = 200
MIN_POSITION_MM = 0

# 計算每 mm 所需步數
steps_per_mm = (STEPS_PER_REV * MICROSTEP) / LEAD_MM_PER_REV

# 狀態參數
pause_flag = threading.Event()
pause_flag.set()
current_position_mm = 200  # 起始位置
step_delay = 0.0003  # 🔧 可透過 set_speed 修改

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(DIR_PIN, GPIO.OUT)
    GPIO.setup(STEP_PIN, GPIO.OUT)
    GPIO.setup(EN_PIN, GPIO.OUT)
    GPIO.output(EN_PIN, GPIO.LOW)
    GPIO.output(DIR_PIN, GPIO.HIGH)
    print("✅ GPIO 初始化完成")
    print("🧠 當前設定 steps_per_mm =", steps_per_mm)

def move_mm(mm, direction="forward"):
    global current_position_mm, step_delay
    total_steps = int(mm * steps_per_mm)

    if direction == "forward":
        if current_position_mm + mm > MAX_POSITION_MM:
            print("❌ 超出最大位置，無法前進")
            return
        GPIO.output(DIR_PIN, GPIO.HIGH)
    elif direction == "backward":
        if current_position_mm - mm < MIN_POSITION_MM:
            print("❌ 超出最小位置，無法後退")
            return
        GPIO.output(DIR_PIN, GPIO.LOW)
    else:
        print("❌ 錯誤方向，請使用 forward 或 backward")
        return

    print(f"⚙️ 開始移動 {direction} {mm}mm（共 {total_steps} 步）")
    for _ in range(total_steps):
        pause_flag.wait()
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(step_delay)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(step_delay)

    current_position_mm += mm if direction == "forward" else -mm
    print(f"✅ 移動完成，目前位置：{current_position_mm:.2f} mm")

def forward_loop():
    global current_position_mm, step_delay
    GPIO.output(DIR_PIN, GPIO.HIGH)
    print("🔁 開始持續前進...（輸入 pause 停止）")
    while pause_flag.is_set():
        if current_position_mm >= MAX_POSITION_MM:
            print("⚠️ 已達最大位置，停止前進")
            break
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(step_delay)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(step_delay)
        current_position_mm += 1 / steps_per_mm
    print("⏹️ 已停止前進")

def backward_loop():
    global current_position_mm, step_delay
    GPIO.output(DIR_PIN, GPIO.LOW)
    print("🔁 開始持續後退...（輸入 pause 停止）")
    while pause_flag.is_set():
        if current_position_mm <= MIN_POSITION_MM:
            print("⚠️ 已達最小位置，停止後退")
            break
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(step_delay)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(step_delay)
        current_position_mm -= 1 / steps_per_mm
    print("⏹️ 已停止後退")

def command_listener():
    global current_position_mm, step_delay
    while True:
        cmd = input("指令（move/go/back/pause/resume/set_speed/status/exit）：").strip().lower()
        if cmd.startswith("move"):
            try:
                parts = cmd.split()
                mm = float(parts[1])
                direction = parts[2] if len(parts) >= 3 else "forward"
                if 0 < mm <= MAX_POSITION_MM:
                    move_mm(mm, direction=direction)
                else:
                    print("❌ 請輸入 0~200mm 的距離")
            except:
                print("❌ 格式錯誤，請用：move 50 backward")
        elif cmd == "go":
            pause_flag.set()
            threading.Thread(target=forward_loop, daemon=True).start()
        elif cmd == "back":
            pause_flag.set()
            threading.Thread(target=backward_loop, daemon=True).start()
        elif cmd == "pause":
            pause_flag.clear()
            print("⏸️ 已暫停")
        elif cmd == "resume":
            pause_flag.set()
            print("▶️ 繼續運行")
        elif cmd.startswith("set_speed"):
            try:
                speed = float(cmd.split()[1])
                if 0.00005 <= speed <= 0.01:
                    step_delay = speed
                    print(f"⚙️ 已更新步進延遲為 {step_delay} 秒")
                else:
                    print("❌ 請輸入合理範圍，例如：set_speed 0.0003")
            except:
                print("❌ 格式錯誤，請用：set_speed 0.0005")
        elif cmd == "status":
            print(f"📍 目前位置：{current_position_mm:.2f} mm | 速度 delay = {step_delay} 秒")
        elif cmd == "exit":
            pause_flag.set()
            print("⚠️ 結束程式，釋放 GPIO")
            GPIO.output(EN_PIN, GPIO.HIGH)
            GPIO.cleanup()
            exit()
        else:
            print("❓ 不明指令，請輸入：move/go/back/pause/resume/set_speed/status/exit")

if __name__ == "__main__":
    setup()
    threading.Thread(target=command_listener, daemon=True).start()
    while True:
        time.sleep(1)
