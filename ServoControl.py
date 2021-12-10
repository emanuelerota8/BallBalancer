from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo
import numpy as np
import time

class ServoControl():
    def __init__(self,pin,clipMin,clipMax):
        self.angle = 0
        self.pin = pin
        self.clipMax = clipMax
        self.clipMin = clipMin

        factory = PiGPIOFactory()
        self.servo = AngularServo(pin, min_angle=90, max_angle=-90,pin_factory=factory,max_pulse_width=0.0024, min_pulse_width=0.0006)
        self.setAngle(0)


    def setAngle(self,angle):
        clippedAngle = np.clip(angle,self.clipMin,self.clipMax)
        self.angle = clippedAngle
        self.servo.angle = clippedAngle


if __name__ =="__main__":
    servoX = ServoControl(17,-90,90)
    servoX.setAngle(-90)
    time.sleep(2)
    servoX.setAngle(-45)
    time.sleep(2)
    servoX.setAngle(0)
    time.sleep(2)
    servoX.setAngle(90)
    time.sleep(2)
    
