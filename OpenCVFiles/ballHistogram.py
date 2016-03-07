# -*- encoding: UTF-8 -*-
# Get an image from NAO. Display it and save it using PIL.

import sys
import time

# Python Image Library
import Image
import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('ball.png')
hist = cv2.calcHist([img],[0],None,[256],[0,256])
hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

# create a mask
mask = np.zeros(img.shape[:2], np.uint8)
mask[370:475, 260:360] = 255
masked_img = cv2.bitwise_and(img,img,mask = mask)

# Calculate histogram with mask and without mask
# Check third argument for mask
hist_full = cv2.calcHist([hsv],[0],None,[256],[0,256])
histh_mask = cv2.calcHist([hsv],[0],mask,[256],[0,256])
hists_mask = cv2.calcHist([hsv],[1],mask,[256],[0,256])
histv_mask = cv2.calcHist([hsv],[2],mask,[256],[0,256])

#plt.subplot(221), plt.imshow(img, 'gray')
plt.subplot(221), plt.imshow(masked_img[:, :, ::-1], 'gray')
plt.subplot(222), plt.plot(histh_mask)
plt.subplot(223), plt.plot(hists_mask)
plt.subplot(224), plt.plot(histv_mask)
plt.xlim([0,256])

plt.show()

cv2.waitKey(0)
