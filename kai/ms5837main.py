from ms5837 import MS5837_02BA
import time

sensor = MS5837_02BA()

if not sensor.init():
    print("❌ 感測器初始化失敗")
    exit()

print("✅ 開始讀取 MS5837 資料")

# ───── 參數設定 ─────
HISTORY_LEN = 5
PRESSURE_THRESHOLD = 1015.0  # 大於此值 → 判定為水中
ENTRY_DIFF = 5.0
FLUID_DENSITY = 997  # 淡水
G = 9.80665          # 重力加速度
# ───────────────────

# 基準壓力（在空氣中記錄）
print("📏 正在校正基準壓力（空氣中）...")
while not sensor.read():
    time.sleep(0.1)
init_pressure = sensor.pressure()
print(f"🎯 基準壓力：{init_pressure:.2f} mbar\n")

pressure_history = []
last_pressure = None
in_water = False  # 初始視為不在水中

while True:
    if sensor.read():
        pressure = sensor.pressure()
        temperature = sensor.temperature()
        
        # 👉 使用基準壓力計算相對水深（單位：m）
        pressure_diff = max(0, (pressure - init_pressure) * 100)  # 單位：Pa，避免負數
        depth = pressure_diff / (FLUID_DENSITY * G)

        # 移動平均
        pressure_history.append(pressure)
        if len(pressure_history) > HISTORY_LEN:
            pressure_history.pop(0)
        avg_pressure = sum(pressure_history) / len(pressure_history)

        # 進出水判斷
        if not in_water and avg_pressure > PRESSURE_THRESHOLD:
            if last_pressure and (pressure - last_pressure) > ENTRY_DIFF:
                print("🟦 判定：剛進入水中")
            in_water = True

        elif in_water and avg_pressure < PRESSURE_THRESHOLD:
            if last_pressure and (last_pressure - pressure) > ENTRY_DIFF:
                print("⬜️ 判定：剛脫離水面")
            in_water = False

        # 輸出顯示
        print(f"壓力：{pressure:.2f} mbar（平均：{avg_pressure:.2f}）")
        print(f"溫度：{temperature:.2f} °C")
        print(f"水深：約 {depth:.3f} m")
        print(f"狀態：{'🔵 在水中' if in_water else '⚪ 在空氣中'}")
        print("────────────────────────────")

        last_pressure = pressure
    else:
        print("⚠️ 感測器讀取失敗")
    time.sleep(1)
