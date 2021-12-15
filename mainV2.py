import cv2
import time
from simple_pid import PID
from ServoControl import ServoControl
from KalmanFiltering import KalmanFiltering
from finder import preprocess, filterOutsidePlate, findball, addCrosshair
import argparse

def main(args):
    # SETTING MAX ANGLE CONTROLLABLE
    CLIP_X_MIN = -15
    CLIP_X_MAX = 15
    CLIP_Y_MIN = -15
    CLIP_Y_MAX = 15

    # define a video capture object
    vid = cv2.VideoCapture(0)  # 2 on laptop

    scale = 0.4  # da abbassare per avere meno problemi di centro,

    servoX = ServoControl(17, CLIP_X_MIN, CLIP_X_MAX)
    servoY = ServoControl(27, CLIP_Y_MIN, CLIP_Y_MAX)

    input("Press enter to continue")

    T = 30/1000  # TODO
    startup = True

    kalmanX = KalmanFiltering(T)
    kalmanY = KalmanFiltering(T)

    # storicoX = []
    # storicoXKalman = []

    precX = 0
    precY = 0

    for i in range(10000):
        _, frame = vid.read()
        timeStart = round(time.time() * 1000)

        # Ball tracking
        frame = preprocess(frame, scale=scale)
        preview = frame if args.video else None
        frame, preview = filterOutsidePlate(frame, debug_img=preview)
        cx, cy, preview = findball(frame, debug_img=preview)
        preview = addCrosshair(preview)

        # DEFINE PID TARGET TO THE CENTER
        xTarget = int(frame.shape[1] /2)
        yTarget = int(frame.shape[0] /2)

        if cx is None:
            cx = precX

        if cy is None:
            cy = precY

        precX = cx
        precY = cy

        cxK = kalmanX.getEstimate(cx)[0]
        cyK = kalmanY.getEstimate(cy)[0]

        print("coords: " + str(cxK) + "  " + str(cyK))

        # override kalman filter
        # cxK=cx
        # cyK=cy

        if startup:
            pidX = PID(.2, 0, 0.05, setpoint=xTarget)
            pidX.output_limits = (CLIP_X_MIN, CLIP_X_MAX)
            pidY = PID(.2, 0, 0.05, setpoint=yTarget)
            pidY.output_limits = (CLIP_Y_MIN, CLIP_Y_MAX)
            startup = False

        if args.video:
            cv2.namedWindow('Tracker', cv2.WINDOW_KEEPRATIO)
            cv2.imshow('Tracker', preview)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        controlX = pidX(cx)
        controlY = pidY(cy)
        print("pid: " + str(controlX))

        # w = 50  # frame.shape[1]*1
        # h = 50  # frame.shape[0]*1
        # controlX = ((controlX - (-w/2))/(w/2+w/2)) * (CLIP_X_MAX - CLIP_X_MIN) + CLIP_X_MIN
        # controlY = ((controlY - (-h/2))/(h/2+h/2)) * (CLIP_Y_MAX - CLIP_Y_MIN) + CLIP_Y_MIN

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

def parse_args():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--video',
        action='store_true',
        help='Enable video preview')

    return argparser.parse_args()


if __name__ == '__main__':
    args = parse_args()
    main(args)
