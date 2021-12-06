from pinInterface import setupServo,goToAngle
import RPi.GPIO as GPIO


CLIP_X_MIN =6.25
CLIP_X_MAX =9.25
CLIP_Y_MIN =5
CLIP_Y_MAX = 9

pY = setupServo(servoPIN = 27)
pX = setupServo(servoPIN = 17)

goToAngle(pX,7,CLIP_X_MIN,CLIP_X_MAX)
goToAngle(pY,7,CLIP_Y_MIN,CLIP_Y_MAX)


print("######### setup X ##############")

valX = 7
cmd = ""

while cmd is not "s":
    cmd = input("Please enter a + or - or s :\n")
    if cmd == "+":
        valX+=0.25
        goToAngle(pX,valX,CLIP_X_MIN,CLIP_X_MAX)
        print(valX)
    else:
        valX -= 0.25
        goToAngle(pX,valX,CLIP_X_MIN,CLIP_X_MAX)
        print(valX)
        
        

print("########## setup Y ###############")

valY = 7
cmd = ""

while cmd is not "s":
    cmd = input("Please enter a + or - or s :\n")
    if cmd == "+":
        valY+=0.25
        goToAngle(pY,valY,CLIP_Y_MIN,CLIP_Y_MAX)
        print(valY)
    else:
        valY -= 0.25
        goToAngle(pY,valY,CLIP_Y_MIN,CLIP_Y_MAX)
        print(valY)



pX.stop()
pY.stop()
GPIO.cleanup()
