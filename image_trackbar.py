# -*- encoding: UTF-8 -*-
# Get an image from NAO. Display it and save it using PIL.

import sys
import time

# Python Image Library
import Image

import argparse
from naoqi import ALProxy

import numpy as np
import cv2


def nothing(x):
    pass


def main():
    frame = cv2.imread('monitor_photo2.jpg')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.namedWindow('Trackbars', cv2.CV_WINDOW_AUTOSIZE)
    cv2.createTrackbar('Hue low', 'Trackbars', 0, 255, nothing)
    # hmin = cv2.getTrackbarPos('Hue low','Trackbars')
    cv2.createTrackbar('Hue high', 'Trackbars', 255, 255, nothing)
    # hmax = cv2.getTrackbarPos('Hue high','Trackbars')
    cv2.createTrackbar('Saturation low', 'Trackbars', 0, 255, nothing)
    # smin = cv2.getTrackbarPos('Saturation low','Trackbars')
    cv2.createTrackbar('Saturation high', 'Trackbars', 255, 255, nothing)
    # smax = cv2.getTrackbarPos('Saturation high','Trackbars')
    cv2.createTrackbar('Value low', 'Trackbars', 0, 255, nothing)
    # vmin = cv2.getTrackbarPos('Value low','Trackbars')
    cv2.createTrackbar('Value high', 'Trackbars', 255, 255, nothing)
    # vmax = cv2.getTrackbarPos('Value max','Trackbars')
    while(1):
        hmin = cv2.getTrackbarPos('Hue low', 'Trackbars')
        hmax = cv2.getTrackbarPos('Hue high', 'Trackbars')
        smin = cv2.getTrackbarPos('Saturation low', 'Trackbars')
        smax = cv2.getTrackbarPos('Saturation high', 'Trackbars')
        vmin = cv2.getTrackbarPos('Value low', 'Trackbars')
        vmax = cv2.getTrackbarPos('Value high', 'Trackbars')
        lower_blue = np.array([hmin, smin, vmin])
        upper_blue = np.array([hmax, smax, vmax])
        # lower_blue = np.array([hmin, 0, 0])
        # upper_blue = np.array([hmax, smax, 255])

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow('frame', frame)
        cv2.imshow('mask', mask)
        cv2.imshow('res', res)

        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
  IP = "192.168.1.107"  # Replace here with your NaoQi's IP address.
  PORT = 9559

  # Read IP address from first argument if any.
  if len(sys.argv) > 1:
    IP = sys.argv[1]

  naoImage = main()
