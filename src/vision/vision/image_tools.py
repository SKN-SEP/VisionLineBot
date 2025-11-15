# Libraries
import numpy as np
import cv2 as cv

# Constants
FIRST_K_SIZE = (15, 15)
SECOND_K_SIZE = (8, 8)
THRES_LOW_EDGES = 40
THRES_HIGH_EDGES = 45

RHO = 1
THETA = np.pi / 180
THRESHOLD = 40
MIN_LINE_LEN = 50

def detect_line(img):
    # First blurr
    blurred_img = cv.blur(img, ksize=FIRST_K_SIZE)

    # Canny edge detection
    canny_img = cv.Canny(blurred_img, THRES_LOW_EDGES, THRES_HIGH_EDGES)

    # Second blurr
    blurred_canny_img = cv.blur(canny_img, ksize=SECOND_K_SIZE)

    # Hough Transform
    lines = cv.HoughLinesP(blurred_canny_img, RHO, THETA, THRESHOLD, minLineLength=MIN_LINE_LEN)

    # Compute average x position of the line
    lines = lines.reshape(-1, 4)
    avg_x = np.average((lines[:, 0] + lines[:, 2]) / 2)

    return avg_x
