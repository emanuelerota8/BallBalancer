from pinInterface import setupServo,goToAngle
import RPi.GPIO as GPIO
import time
import numpy as np


CLIP_X_MIN =6.25
CLIP_X_MAX =9.25
CLIP_Y_MIN =5
CLIP_Y_MAX = 9


GPIO.setmode(GPIO.BCM)
GPIO.setup(17,GPIO.OUT)

p = GPIO.PWM(17,50)
p.start(0)

data = np.ones((500,500,3))

for i in range(250):
    time.sleep(1/30)
    p.ChangeDutyCycle(6)
    time.sleep(1/30)
    p.ChangeDutyCycle(6)

    #time.sleep(1)
    




p.stop()

GPIO.cleanup()
