import cv2
import time
from simple_pid import PID
from ServoControl import ServoControl
from KalmanFiltering import KalmanFiltering
from finder import preprocess, filterOutsidePlate, findball, addCrosshair
import argparse
import numpy as np
import math
import pickle

def main(args):
    # SETTING MAX ANGLE CONTROLLABLE
    CLIP_X_MIN = -20
    CLIP_X_MAX = 20
    CLIP_Y_MIN = -20
    CLIP_Y_MAX = 20

    #control history
    controlHistoryX = []
    controlHistoryY = []
    posXhist = []
    posYhist = []

    # define a video capture object
    vid = cv2.VideoCapture(0)  # 2 on laptop

    servoX = ServoControl(17, CLIP_X_MIN, CLIP_X_MAX)
    servoY = ServoControl(27, CLIP_Y_MIN, CLIP_Y_MAX)

    input("Press enter to continue")

    T = 1000/30
    startup = True

    kalmanX = KalmanFiltering(T)
    kalmanY = KalmanFiltering(T)

    precX = 0
    precY = 0

    for i in range(10000):
        _, frame = vid.read()
        timeStart = time.time()

        # Ball tracking
        frame = preprocess(frame, scale=args.scale)
        preview = frame if args.video else None
        frame, preview = filterOutsidePlate(frame, debug_img=preview)
        cx, cy, preview = findball(frame, debug_img=preview)

        # DEFINE PID TARGET TO THE CENTER
        xTarget = int(frame.shape[1] /2)
        yTarget = int(frame.shape[0] /2)
        preview = addCrosshair(preview, x=xTarget, y=yTarget)

        if cx is None:
            cx = precX

        if cy is None:
            cy = precY

        # Kalman prediction
        if args.nokalman != True:
            cx = kalmanX.getEstimate(cx)[0]
            cy = kalmanY.getEstimate(cy)[0]
        preview = addCrosshair(preview, x=cx, y=cy, color=(0,0,255), size=.05)

        
        precX = cx
        precY = cy
        
        if args.circle:
            Kp = 0.03*1
            Ki = 0.012*3
            Kd = 0.01875*3.5
        else:
            PROP = 1.1
            Kp = 0.03*.7*PROP
            Ki = 0.012*2.2*PROP
            Kd = 0.01875*3.5*PROP

        if startup:
            pidX = PID(Kp, Ki, Kd, setpoint=xTarget,sample_time=T/1000)
            pidX.output_limits = (CLIP_X_MIN, CLIP_X_MAX)
            pidY = PID(Kp, Ki, Kd, setpoint=yTarget,sample_time=T/1000)
            pidY.output_limits = (CLIP_Y_MIN, CLIP_Y_MAX)
            startup = False
            angle = 0

        if args.circle:
            angle +=1
            angle = int(angle)
            angle = angle%360
            rad = angle*3.14/180
            radius = 50-17

            newX = xTarget + math.cos(rad)*radius
            newY = yTarget + math.sin(rad)*radius

            pidX.setpoint = newX
            pidY.setpoint = newY

        if args.video:
            cv2.namedWindow('Tracker', cv2.WINDOW_KEEPRATIO)
            cv2.imshow('Tracker', preview)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Apply control
        controlX = pidX(cx)
        controlY = pidY(cy)

        controlHistoryX.append(controlX)
        controlHistoryY.append(controlY)

        if len(controlHistoryX) >=5:
            filteredControlX = np.mean(controlHistoryX[-2:-1])
            filteredControlY = np.mean(controlHistoryY[-2:-1])
        else:
            filteredControlX=controlX
            filteredControlY=controlY

        servoX.setAngle(filteredControlX)
        servoY.setAngle(filteredControlY)

        deltaTime = max( 1/29 - (time.time() - timeStart),0)
        
        time.sleep(deltaTime)

        fps = 1 / (time.time() - timeStart)

        posXhist.append(cx)
        posYhist.append(cy)

        if i % 60==0:
            with open("history.pkl", "wb") as f:
                pickle.dump([posXhist, posYhist], f)

        print(f'FPS={fps:.1f} BALL=({cx:.2f}, {cy:.2f}) PID_CONTROL=({controlX:.2f}, {controlY:.2f})')

    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

def parse_args():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--video',
        action='store_true',
        help='Enable video preview')
    argparser.add_argument(
        '--nokalman',
        action='store_true',
        help='disable kalman filter')
    argparser.add_argument(
        '--circle',
        action='store_true',
        help='draw circle')
    argparser.add_argument(
        '--scale',
        type=float,
        default=0.4,
        help='Camera downscaling')


    return argparser.parse_args()


if __name__ == '__main__':
    args = parse_args()
    main(args)
