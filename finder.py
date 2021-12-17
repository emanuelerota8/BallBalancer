import cv2
import numpy as np
import time
from KalmanFiltering import KalmanFiltering
import matplotlib.pyplot as plt
import pickle


def preprocess(img, scale=.5):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.resize(
        img, (int(img.shape[1] * scale), int(img.shape[0] * scale)))
    img = cv2.GaussianBlur(img, (3, 3), sigmaX=0, sigmaY=0)
    return img


def filterOutsidePlate(img, debug_img=None):
    # Highlight plate in white with thick black edges
    mask = cv2.Canny(img, 35, 35/3)
    kernel = np.ones((8, 8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
    mask = np.bitwise_not(mask)
    # Get convex hull of the biggest white area
    cnts, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
    cnt = cnts[0]
    cnt = cv2.convexHull(cnt, False)
    # Set everything outside the plate as pure white
    mask = cv2.drawContours(mask * 0, [cnt], -1, 255, -1)
    # Render image for debugging purposes
    if debug_img is not None:
        if len(debug_img.shape) == 2:
            debug_img = cv2.cvtColor(debug_img, cv2.COLOR_GRAY2BGR)
        plate_area = debug_img * 0
        plate_area[mask == 255] = [0, 255, 75]
        debug_img = cv2.addWeighted(debug_img, 1, plate_area, .5, 0)
    img[mask != 255] = 255
    return img, debug_img


def findball(img, debug_img=None):
    # Find darkest pixels and erode them a bit to
    # cancel out any line or stick inside the image
    _, img = cv2.threshold(img, np.median(img)*.25, 255, cv2.THRESH_BINARY_INV)
    kernel = np.ones((3, 3))
    img = cv2.morphologyEx(img, cv2.MORPH_ERODE, kernel)
    # Find average pixel coords, if there is any
    idxs = np.argwhere(img == 255)
    cx = cy = None
    if len(idxs) > 0:
        cy, cx = np.mean(idxs, axis=0)
        # Render image  with crosshair for debugging purposes
        if debug_img is not None:
            if len(debug_img.shape) == 2:
                debug_img = cv2.cvtColor(debug_img, cv2.COLOR_GRAY2BGR)
            ball_area = debug_img * 0
            debug_img[img == 255] = [255, 0,  0]
            # debug_img = cv2.addWeighted(debug_img, 1, ball_area, 1, 0)
            debug_img = cv2.line(debug_img, (int(cx), 0), (int(
                cx), debug_img.shape[0]), (255, 255, 255), thickness=2)
            debug_img = cv2.line(debug_img, (0, int(
                cy)), (debug_img.shape[1], int(cy)), (255, 255, 255), thickness=2)
        # cx /= img.shape[1]
        # cy /= img.shape[0]

    return cx, cy, debug_img


def addCrosshair(img, x=0.5, y=0.5, size=.1, color=(255, 255, 255)):
    if img is not None:
        cy = int(y)  # int(img.shape[0] * y)
        cx = int(x)  # int(img.shape[1] * x)
        sz = int(max(img.shape) * size)
        img = cv2.line(img, (cx-sz, cy), (cx+sz, cy), color, thickness=2)
        img = cv2.line(img, (cx, cy-sz), (cx, cy+sz), color, thickness=2)
    return img


def main():
    cap = cv2.VideoCapture('sample_videos/test1.mkv')

    historyX = []
    historyY = []
    historyXK = []
    historyYK = []

    T = 1000/30
    kalmanX = KalmanFiltering(T)
    kalmanY = KalmanFiltering(T)

    i = 0
    while cap.isOpened():
        _, frame = cap.read()

        if frame is None:
            break
        frame = preprocess(frame)

        i += 1
        start_time = time.time()
        cv2.imshow('window-name', frame)
        frame, preview = filterOutsidePlate(frame, debug_img=frame)

        cx, cy, preview = findball(frame, debug_img=preview)

        cxK = kalmanX.getEstimate(cx)[0]
        cyK = kalmanY.getEstimate(cy)[0]

        historyX.append(cx)
        historyY.append(cy)
        historyXK.append(cxK)
        historyYK.append(cyK)

        preview = addCrosshair(preview)
        cv2.imshow('window-name2', preview)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        print(f'Frame {i} FPS: ', 1.0 / (time.time() - start_time))

    cap.release()
    cv2.destroyAllWindows()  # destroy all opened windows

    with open("x", "wb") as f:
        pickle.dump([historyX, historyY, historyXK, historyYK], f)


if __name__ == "__main__":
    main()
