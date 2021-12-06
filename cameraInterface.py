import cv2
import time
import numpy as np
from simple_pid import PID
import RPi.GPIO as GPIO
from pinInterface import setupServo,goToAngle

CLIP_X_MIN =6.25 +1
CLIP_X_MAX =9.25 -1
CLIP_Y_MIN =5 +1
CLIP_Y_MAX = 9 -1

# define a video capture object
vid = cv2.VideoCapture(0)#2 on laptop

scale = 0.1
minRadius = int(np.ceil(10 * scale))
maxRadius = int(np.ceil(20 * scale))

xCenter = []
yCenter = []

actualX = 0
actualY = 0

pY = setupServo(servoPIN = 27)
pX = setupServo(servoPIN = 17)

startup = True

while True:
    ret, frame = vid.read()
    
    frame = frame[90:480,180:610]
   
    #print(frame.shape)

    frame = cv2.resize(
        frame, (int(frame.shape[1]*scale), int(frame.shape[0]*scale)))
    
    xTarget = int(frame.shape[1] /2)
    yTarget = int(frame.shape[0] /2)
    
    if startup:
        pidX = PID(1, 0.3, 3, setpoint=xTarget)
        pidY = PID(1, 0.3, 3, setpoint=yTarget)
        startup=False
    
    #cv2.imwrite("img.png",frame)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #bilateral = cv2.bilateralFilter(gray, 5, 5, 75)
    bilateral = gray
    
    bilateral[bilateral <= 30] = 0
    bilateral[bilateral >= 80] = 255
    bilateral = cv2.bitwise_not(bilateral)
    
    # cv2.namedWindow('b', cv2.WINDOW_KEEPRATIO)
    # cv2.imshow('b', bilateral)

    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

    edges = cv2.Canny(image=bilateral, threshold1=100,
                      threshold2=200)  # Canny Edge

    circles = cv2.HoughCircles(image=edges,
                               method=cv2.HOUGH_GRADIENT,
                               dp=1.5,
                               minDist=2*minRadius,
                               param1=1,
                               param2=25-15 - 5,
                               minRadius=minRadius,
                               maxRadius=maxRadius
                               )

    if circles is not None:
        circles = circles[0, 0]
        xCenter.append(int(circles[0]))
        yCenter.append(int(circles[1]))
        # processed = cv2.circle(
        #     frame, (int(circles[0]), int(circles[1])), int(circles[2]), (255, 0, 0), 2)

        actualX = circles[0]
        actualY = circles[1]

        if False:
            for idx in range(len(xCenter)):
                processed = cv2.circle(
                    frame, (int(xCenter[idx]), int(yCenter[idx])), 1, (255, 255, 0), 2)

    else:
        print("Ball not found")
        processed = frame

    processed = cv2.circle(
        frame, (int(actualX), int(actualY)), 1, (255, 255, 0), 2)
    
    #PID controller
    
    controlX = - pidX(actualX)
    controlY = - pidY(actualY)
    
    
    w = processed.shape[1]
    h = processed.shape[0]

    trueActionX =((controlX - (-w/2))/(w/2+w/2) )* (CLIP_X_MAX - CLIP_X_MIN) + CLIP_X_MIN
    trueActionY =((controlY - (-h/2))/(h/2+h/2) )* (CLIP_Y_MAX - CLIP_Y_MIN) + CLIP_Y_MIN
    
    goToAngle(pX,trueActionX,CLIP_X_MIN,CLIP_X_MAX)
    goToAngle(pY,trueActionY,CLIP_Y_MIN,CLIP_Y_MAX)
    print("true actionn" + str(trueActionX))


    # cv2.namedWindow('frame', cv2.WINDOW_KEEPRATIO)
    # cv2.imshow('frame', processed)

    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break


# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()

pX.stop()
pY.stop()
GPIO.cleanup()
