import RPi.GPIO as GPIO
import time

ITERATIONS = 2
LOWER = 6.5
UPPER = 8.5
START = 7.5
SLEEP = 0.5

def setupServo():
    servoPIN = 17
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(servoPIN,GPIO.OUT)

    p = GPIO.PWM(servoPIN,50)
    p.start(0)
    time.sleep(0.1)
    return p

def rotateOverFullRange(p,increment):
    i=0
    count = START
    if False:
        while i< ITERATIONS:
            count+=1
            print(count)
            p.ChangeDutyCycle(count)
            time.sleep(0.5)
            if count == START:
                i+=increment
            if count >= UPPER or count <=LOWER:
                increment*=-1

def goToAngle(p,angle):
    p.ChangeDutyCycle(angle)
    time.sleep(SLEEP)
    pass



if __name__ == "__main__":
    p = setupServo()
    #rotateOverFullRange(p,1):
    for i in range(3):
        goToAngle(p,LOWER)
        goToAngle(p,START)
        goToAngle(p,UPPER)



    time.sleep(1)
    p.stop()
    GPIO.cleanup()
