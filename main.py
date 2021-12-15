import cv2
import time
import numpy as np
from simple_pid import PID
import RPi.GPIO as GPIO
from pinInterface import setupServo,goToAngle
from ServoControl import ServoControl
from KalmanFiltering import KalmanFiltering
from finder import preprocess, filterOutsidePlate, findball


CLIP_X_MIN =-15
CLIP_X_MAX =15
CLIP_Y_MIN =-15
CLIP_Y_MAX = 15

# define a video capture object
vid = cv2.VideoCapture(0)#2 on laptop

scale = 0.5
minRadius = int(np.ceil(10 * scale))
maxRadius = int(np.ceil(20 * scale))

xCenter = []
yCenter = []

actualX = 0
actualY = 0

servoX = ServoControl(17,CLIP_X_MIN,CLIP_X_MAX)
servoY = ServoControl(27,CLIP_Y_MIN,CLIP_Y_MAX)

input("Press enter to continue")

startup = True
first = True
misurazioniX = []

kalmanX = KalmanFiltering(1/30)
kalmanY = KalmanFiltering(1/30)

while True:
    ret, frame = vid.read()
    
    if False:
        frame = frame[40:400,140:470]

    frame = cv2.resize(
        frame, (int(frame.shape[1]*scale), int(frame.shape[0]*scale)))

    # cv2.namedWindow('full', cv2.WINDOW_KEEPRATIO)
    # cv2.imshow('full', frame)

    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
    
    frame = preprocess(frame)
    frame,_ = filterOutsidePlate(frame)
    cx,cy ,frame= findball(frame)

    xTarget = int(frame.shape[1] /2)
    yTarget = int(frame.shape[0] /2)
    
    if startup:
        pidX = PID(0.9, 0.2, 0.9, setpoint=xTarget)
        pidY = PID(0.9, 0.2, 0.9, setpoint=yTarget)
        startup=False

    
    
    # cv2.namedWindow('b', cv2.WINDOW_KEEPRATIO)
    # cv2.imshow('b', frame)

    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

    if first == True:
        actualXK = kalmanX.getEstimate(cx)[0]
        actualYK = kalmanY.getEstimate(cy)[0]
        first = False

    if cx is not None:
        actualXK = kalmanX.getEstimate(cx)[0]
        actualYK = kalmanY.getEstimate(cy)[0]


    
        # misurazioniX.append(actualY)
        # print("std",np.std(misurazioniX))

        processed = cv2.circle(
            frame, (int(cx), int(cy)), 1, (255, 255, 0), 2)

        processed = cv2.circle(
            processed, (int(actualXK), int(actualYK)), 1, (0, 255, 0), 2)
    
    #PID controller
    
    controlX = pidX(actualXK)
    controlY =  pidY(actualYK)
    
    
    w = processed.shape[1]
    h = processed.shape[0]

    trueActionX =((controlX - (-w/2))/(w/2+w/2) )* (CLIP_X_MAX - CLIP_X_MIN) + CLIP_X_MIN
    trueActionY =((controlY - (-h/2))/(h/2+h/2) )* (CLIP_Y_MAX - CLIP_Y_MIN) + CLIP_Y_MIN

    servoX.setAngle(trueActionX)
    servoY.setAngle(trueActionY)

    #print(trueActionX)
    

    #print("true actionn" + str(trueActionX))


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
