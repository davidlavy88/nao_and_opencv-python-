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


def main(_name):
    frame = cv2.imread(_name)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.namedWindow('Trackbars', cv2.CV_WINDOW_AUTOSIZE)
    cv2.createTrackbar('Hue 1 low', 'Trackbars', 0, 255, nothing)
    # hmin = cv2.getTrackbarPos('Hue low','Trackbars')
    cv2.createTrackbar('Hue 1 high', 'Trackbars', 255, 255, nothing)
    # hmax = cv2.getTrackbarPos('Hue high','Trackbars')
    cv2.createTrackbar('Hue 2 low', 'Trackbars', 0, 255, nothing)
    # hmin = cv2.getTrackbarPos('Hue low','Trackbars')
    cv2.createTrackbar('Hue 2 high', 'Trackbars', 255, 255, nothing)
    cv2.createTrackbar('Saturation low', 'Trackbars', 0, 255, nothing)
    # smin = cv2.getTrackbarPos('Saturation low','Trackbars')
    cv2.createTrackbar('Saturation high', 'Trackbars', 255, 255, nothing)
    # smax = cv2.getTrackbarPos('Saturation high','Trackbars')
    cv2.createTrackbar('Value low', 'Trackbars', 0, 255, nothing)
    # vmin = cv2.getTrackbarPos('Value low','Trackbars')
    cv2.createTrackbar('Value high', 'Trackbars', 255, 255, nothing)
    # vmax = cv2.getTrackbarPos('Value max','Trackbars')
    while(1):
        hmin1 = cv2.getTrackbarPos('Hue 1 low', 'Trackbars')
        hmax1 = cv2.getTrackbarPos('Hue 1 high', 'Trackbars')
        hmin2 = cv2.getTrackbarPos('Hue 2 low', 'Trackbars')
        hmax2 = cv2.getTrackbarPos('Hue 2 high', 'Trackbars')
        smin = cv2.getTrackbarPos('Saturation low', 'Trackbars')
        smax = cv2.getTrackbarPos('Saturation high', 'Trackbars')
        vmin = cv2.getTrackbarPos('Value low', 'Trackbars')
        vmax = cv2.getTrackbarPos('Value high', 'Trackbars')
        lower_blue = np.array([hmin1, smin, vmin])
        upper_blue = np.array([hmax1, smax, vmax])
        lower_red = np.array([hmin2, smin, vmin])
        upper_red = np.array([hmax2, smax, vmax])
        # lower_blue = np.array([hmin, 0, 0])
        # upper_blue = np.array([hmax, smax, 255])

        # Threshold the HSV image to get only blue colors
        mask1 = cv2.inRange(hsv, lower_blue, upper_blue)
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = cv2.add(mask1,mask2)

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
  IP = "192.168.1.122"  # Replace here with your NaoQi's IP address.
  PORT = 9559

  # Read IP address from first argument if any.
  if len(sys.argv) > 1:
    img = sys.argv[1]

  naoImage = main(img)
