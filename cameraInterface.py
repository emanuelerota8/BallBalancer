import cv2
import time
import numpy as np
from simple_pid import PID
from pinInterface import setupServo,goToAngle

# define a video capture object
vid = cv2.VideoCapture(0)#2 on laptop

scale = 0.3
minRadius = int(np.ceil(10 * scale))
maxRadius = int(np.ceil(20 * scale))

xCenter = []
yCenter = []

actualX = 0
actualY = 0

p = setupServo()

while True:
    ret, frame = vid.read()
    
    frame = frame[100:400,100:550]
    #print(frame.shape)

    frame = cv2.resize(
        frame, (int(frame.shape[1]*scale), int(frame.shape[0]*scale)))
    

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    bilateral = cv2.bilateralFilter(gray, 5, 5, 75)

    edges = cv2.Canny(image=bilateral, threshold1=100,
                      threshold2=200)  # Canny Edge

    circles = cv2.HoughCircles(image=edges,
                               method=cv2.HOUGH_GRADIENT,
                               dp=1.5,
                               minDist=2*minRadius,
                               param1=1,
                               param2=25-15,
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
    pid = PID(1, 0.1, 0.05, setpoint=68)
    control = pid(actualX)
    print("control: "+ str(control))
    print("actual: "+ str(actualX))
    print("diff: "+ str(actualX-control))
    
    
    
    trueAction =((control - (-100))/(100+100) )* (2)
    goToAngle(p,trueAction)
    print("true actionn" + str(trueAction))


    cv2.namedWindow('frame', cv2.WINDOW_KEEPRATIO)
    cv2.imshow('frame', processed)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
