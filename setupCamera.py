import cv2
import numpy as np

vid = cv2.VideoCapture(0)#2 on laptop

ret, frame = vid.read()

STEP = 10

yTop = 0
xTop = 0

yBot = frame.shape[0]
xBot = frame.shape[1]

while True:
    ret, frame = vid.read()
    
    frame = frame[yTop:yBot,xTop:xBot]
    #print(frame.shape)
    
    cv2.namedWindow('frame', cv2.WINDOW_KEEPRATIO)
    cv2.imshow('frame', frame)
    
    key = cv2.waitKey(1)
    
    if key  & 0xFF == ord('q'):
        break
    elif key & 0xFF == ord('a'):
        xTop -= STEP
    elif key & 0xFF == ord('d'):
        xTop += STEP
    elif key & 0xFF == ord('w'):
        yTop += STEP
    elif key & 0xFF == ord('s'):
        yTop -= STEP
    elif key & 0xFF == ord('j'):
        xBot -= STEP
    elif key & 0xFF == ord('l'):
        xBot += STEP
    elif key & 0xFF == ord('i'):
        yBot += STEP
    elif key & 0xFF == ord('k'):
        yBot -= STEP
    
    print(yTop,yBot,xTop,xBot)
    

    