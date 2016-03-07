import cv2
import numpy as np

def nothing(x):
    pass

# Create a black image, a window
img = cv2.imread('ball.png')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('Hmin','image',0,255,nothing)
cv2.createTrackbar('Hmax','image',0,255,nothing)
cv2.createTrackbar('Smin','image',0,255,nothing)
cv2.createTrackbar('Smax','image',0,255,nothing)
cv2.createTrackbar('Vmin','image',0,255,nothing)
cv2.createTrackbar('Vmax','image',0,255,nothing)

while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

    # get current positions of four trackbars
    hmin = cv2.getTrackbarPos('Hmin','image')
    hmax = cv2.getTrackbarPos('Hmax','image')
    smin = cv2.getTrackbarPos('Smin','image')
    smax = cv2.getTrackbarPos('Smax','image')
    vmin = cv2.getTrackbarPos('Vmin','image')
    vmax = cv2.getTrackbarPos('Vmax','image')

    # define range of blue color in HSV
    lower_values = np.array([hmin,smin,vmin])
    upper_values = np.array([hmax,smax,vmax])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_values, upper_values)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(img,img, mask= mask)

    cv2.imshow('res',res)

cv2.destroyAllWindows()
