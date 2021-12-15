import cv2
import time
import numpy as np
from simple_pid import PID
import RPi.GPIO as GPIO
from pinInterface import setupServo,goToAngle
from ServoControl import ServoControl
from KalmanFiltering import KalmanFiltering
from finder import preprocess, filterOutsidePlate, findball


#SETTING MAX ANGLE CONTROLLABLE
CLIP_X_MIN =-15
CLIP_X_MAX =15
CLIP_Y_MIN =-15
CLIP_Y_MAX = 15

# define a video capture object
vid = cv2.VideoCapture(0)#2 on laptop

scale = 0.5 #da abbassare per avere meno problemi di centro,


actualX = 0
actualY = 0

servoX = ServoControl(17,CLIP_X_MIN,CLIP_X_MAX)
servoY = ServoControl(27,CLIP_Y_MIN,CLIP_Y_MAX)

input("Press enter to continue")

DEBUG = True

T = 30/1000 #TODO
startup = True

kalmanX = KalmanFiltering(T)
kalmanY = KalmanFiltering(T)

# storicoX = []
# storicoXKalman = []

precX =0
precY=0

for i in range(10000):
    ret, frame = vid.read()
    timeStart = round(time.time() * 1000)

    #decrese resolution
    frame = cv2.resize(
        frame, (int(frame.shape[1]*scale), int(frame.shape[0]*scale)))

    #DEFINE PID TARGET TO THE CENTER
    xTarget = 140#int(frame.shape[1] /2)
    yTarget = 170#int(frame.shape[0] /2)


    if DEBUG:
        cv2.namedWindow('cameraView', cv2.WINDOW_KEEPRATIO)
        cv2.imshow('cameraView', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    #PREPROCESSING AND BALL COORDINATES
    frame = preprocess(frame)

    if DEBUG:
        cv2.namedWindow('pre', cv2.WINDOW_KEEPRATIO)
        cv2.imshow('pre', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    frame,_ = filterOutsidePlate(frame)

    if DEBUG:
        cv2.namedWindow('filtered', cv2.WINDOW_KEEPRATIO)
        cv2.imshow('filtered', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cx,cy ,frame= findball(frame)
    #storicoX.append(cx)

    if cx is  None:
        cx=precX

    if cy is None:
        cy=precY

    precX=cx
    precY=cy

    cxK = kalmanX.getEstimate(cx)[0]
    cyK = kalmanX.getEstimate(cy)[0]

    print("coords: " + str(cxK)+ "  " + str(cyK))

    #override kalman filter
    # cxK=cx
    # cyK=cy

    if startup:
        pidX = PID(1, 0, 0.1, setpoint=xTarget)
        pidY = PID(1, 0, 0.1, setpoint=yTarget)
        startup=False

    if DEBUG:
        cv2.namedWindow('coordinates', cv2.WINDOW_KEEPRATIO)
        cv2.imshow('coordinates', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    controlX = pidX(cxK)
    controlY =  pidY(cyK)
    print("pid: "+str(controlX))


    w = 50 #frame.shape[1]*1
    h = 50 # frame.shape[0]*1
    controlX =((controlX - (-w/2))/(w/2+w/2) )* (CLIP_X_MAX - CLIP_X_MIN) + CLIP_X_MIN
    controlY =((controlY - (-h/2))/(h/2+h/2) )* (CLIP_Y_MAX - CLIP_Y_MIN) + CLIP_Y_MIN

    servoX.setAngle(controlX)
    servoY.setAngle(controlY)

    print("mapped"+str(controlX))

    timeEnd = round(time.time() * 1000)
    print("framerate: "+str(timeEnd-timeStart))

# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()

# np.save("storico",storicoX)
# np.save("kalman",storicoXKalman)