# -*- encoding: UTF-8 -*-
# Get an image from NAO. Display it and save it using PIL.

import sys
import time

# Python Image Library
import Image
import cv2
import cv2.cv as cv
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('ball.png')
# Convert BGR to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# define range of blue color in HSV
lower_valuespos = np.array([0,217,0])
upper_valuespos = np.array([2,240,255])

# define range of blue color in HSV
lower_valuesneg = np.array([177,217,0])
upper_valuesneg = np.array([180,240,255])

# Threshold the HSV image to get only blue colors
mask1 = cv2.inRange(hsv, lower_valuespos, upper_valuespos)
mask2 = cv2.inRange(hsv, lower_valuesneg, upper_valuesneg)

mask = cv2.add(mask1,mask2)

kernel = np.ones((5,5),np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

# Bitwise-AND mask and original image
res = cv2.bitwise_and(img,img, mask= mask)
grayim = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
cv2.imshow('Grayscale',mask)
cv2.waitKey(1000)

circles = cv2.HoughCircles(mask,cv.CV_HOUGH_GRADIENT,1,20,
                            param1=100,param2=30,minRadius=5,maxRadius=150)

if circles is None:
    print "There is no circles"
else:
    circles = np.uint16(np.around(circles))
for i in circles[0,:]:
    # draw the outer circle
    cv2.circle(grayim,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv2.circle(grayim,(i[0],i[1]),2,(0,0,255),3)

#cv2.imshow('detected circles',cimg)

cv2.imshow('frame',img)
cv2.imshow('mask',mask)
cv2.imshow('res',res)
cv2.imshow('circles',grayim)

cv2.waitKey(0)
cv2.destroyAllWindows()
