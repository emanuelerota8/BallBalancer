import cv2
import numpy as np
import time


def preprocess(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.GaussianBlur(img, (3, 3), sigmaX=0, sigmaY=0)
    return img


def filterOutsidePlate(img, debug: bool = False):
    orig = img.copy()
    # Canny
    img = cv2.Canny(img, 35, 35/3)
    # Dilate
    kernel = np.ones((8, 8))
    img = cv2.morphologyEx(img, cv2.MORPH_DILATE, kernel)
    img = np.bitwise_not(img)
    # Find biggest contour
    cnts, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
    cnt = cnts[0]
    cnt = cv2.convexHull(cnt, False)
    # Draw mask
    mask = cv2.drawContours(img * 0, [cnt], -1, 255, -1)
    # mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

    # draw preview
    preview = None
    if debug:
        preview = orig.copy()
        preview = cv2.cvtColor(preview, cv2.COLOR_GRAY2BGR)
        preview = cv2.drawContours(preview, [cnt], -1, (0, 255, 75), -1)
    orig[mask != 255] = 0
    return orig, preview


def findball(img):
    img[img == 0] = 255
    _, img = cv2.threshold(img, np.median(img)*.5, 255, cv2.THRESH_BINARY_INV)

    kernel = np.ones((8, 8))
    img = cv2.morphologyEx(img, cv2.MORPH_ERODE, kernel)

    idxs = np.argwhere(img == 255)

    if len(idxs) > 0:
        cy, cx = [int(val) for val in np.mean(idxs, axis=0)]
        img = cv2.line(img, (cx, 0), (cx, img.shape[0]), (255, 255, 255), thickness=2)
        img = cv2.line(img, (0, cy), (img.shape[1], cy), (255, 255, 255), thickness=2)

    return cx, cy, img


def main():
    cap = cv2.VideoCapture('piatto2.mkv')

    i = 0
    while cap.isOpened():
        _, frame = cap.read()
        # frame = cv2.resize(frame, (200, 200))
        frame = preprocess(frame)

        i += 1
        start_time = time.time()
        cv2.imshow('window-name', frame)
        frame, preview = filterOutsidePlate(frame, debug=True)

        cx, cy, preview = findball(frame)

        cv2.imshow('window-name2', preview)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        print(f'Frame {i} FPS: ', 1.0 / (time.time() - start_time))

    cap.release()
    cv2.destroyAllWindows()  # destroy all opened windows


if __name__ == "__main__":
    main()
