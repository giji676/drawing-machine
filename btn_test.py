import RPi.GPIO as GPIO
import time


GPIO.cleanup()
btns = [7,11,13,15]
leds = [36,38,40]

def setup():
    GPIO.setmode(GPIO.BOARD)
    for btn in btns:
        GPIO.setup(btn, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    for led in leds:
        GPIO.setup(led, GPIO.OUT)
        GPIO.output(led, GPIO.LOW)

setup()

while True:
    if not(GPIO.input(btns[0])):
        print("pressed", btns[0])
    if not(GPIO.input(btns[1])):
        print("pressed", btns[1])
    if not(GPIO.input(btns[2])):
        print("pressed", btns[2])
    if not(GPIO.input(btns[3])):
        print("pressed", btns[3])
    time.sleep(0.01)
