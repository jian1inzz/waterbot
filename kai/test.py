import RPi.GPIO as GPIO
import time

ECHO_SIM = 24  

GPIO.setmode(GPIO.BCM)
GPIO.setup(ECHO_SIM, GPIO.OUT)

try:
    while True:
        GPIO.output(ECHO_SIM, True)   
        print("Echo 模擬高電位 (3.3V)...")
        time.sleep(10) 

        GPIO.output(ECHO_SIM, False)  
        print("Echo 模擬低電位 (0V)...")
        time.sleep(2)  

except KeyboardInterrupt:
    GPIO.cleanup()
