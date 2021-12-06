from pinInterface import setupServo,goToAngle
import RPi.GPIO as GPIO

pY = setupServo(servoPIN = 27)
pX = setupServo(servoPIN = 17)

goToAngle(pX,7)
goToAngle(pY,7)


print("######### setup X ##############")

valX = 7
cmd = ""

while cmd is not "s":
    cmd = input("Please enter a + or - or s :\n")
    if cmd == "+":
        valX+=0.25
        goToAngle(pX,valX)
        print(valX)
    else:
        valX -= 0.25
        goToAngle(pX,valX)
        print(valX)
        
        

print("########## setup Y ###############")

valY = 7
cmd = ""

while cmd is not "s":
    cmd = input("Please enter a + or - or s :\n")
    if cmd == "+":
        valY+=0.25
        goToAngle(pY,valY)
        print(valY)
    else:
        valY -= 0.25
        goToAngle(pY,valY)
        print(valY)



pX.stop()
pY.stop()
GPIO.cleanup()
