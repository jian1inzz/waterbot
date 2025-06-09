import RPi.GPIO as GPIO
import time

TRIG = 23  # 你接 TRIG 的 GPIO 腳位號碼
ECHO = 24  # 你接 ECHO 的 GPIO 腳位號碼

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    # 發送 10 微秒的脈衝信號
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # 等待 Echo 高電位開始
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    # 等待 Echo 回到低電位
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    # 計算時間差
    pulse_duration = pulse_end - pulse_start

    # 計算距離（音速 = 34300 cm/s）
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance

try:
    while True:
        dist = get_distance()
        print(f"距離：{dist} cm")
        time.sleep(0.5)

except KeyboardInterrupt:
    GPIO.cleanup()