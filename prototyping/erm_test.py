import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
ERM_PINS = [2,3,4,17,27,22,10,9,11,5,6,13,19,26]

for erm in ERM_PINS:
    GPIO.setup(erm, GPIO.OUT, initial=GPIO.LOW)


for erm in ERM_PINS:
    print(f"Triggering erm {erm}")
    GPIO.output(erm, GPIO.HIGH)

time.sleep(2)

for erm in ERM_PINS:
    print(f"Triggering erm {erm}")
    GPIO.output(erm, GPIO.LOW)

time.sleep(1)

for erm in ERM_PINS:
    print(f"Triggering erm {erm}")
    GPIO.output(erm, GPIO.HIGH)

time.sleep(2)

for erm in ERM_PINS:
    print(f"Triggering erm {erm}")
    GPIO.output(erm, GPIO.LOW)

time.sleep(1)