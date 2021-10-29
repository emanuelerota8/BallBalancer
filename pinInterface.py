import RPi.GPIO as GPIO
import time


servoPIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN,GPIO.OUT)

p = GPIO.PWM(servoPIN,50)
p.start(0)

if True:
    for i in range(0,1,1):
        time.sleep(1)
        p.ChangeDutyCycle(5)
        time.sleep(0.5)
        p.ChangeDutyCycle(7.5)
        time.sleep(0.5)
        p.ChangeDutyCycle(10)
        time.sleep(0.5)
        p.ChangeDutyCycle(12.5)
        time.sleep(0.5)
        p.ChangeDutyCycle(10)
        time.sleep(0.5)
        p.ChangeDutyCycle(7.5)
        time.sleep(0.5)
        p.ChangeDutyCycle(5)
        time.sleep(0.5)
        p.ChangeDutyCycle(2.5)
        time.sleep(0.5)
      
        #p.ChangeDutyCycle(5)

p.stop()
GPIO.cleanup()