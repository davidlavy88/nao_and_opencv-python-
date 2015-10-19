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

def showNaoImage(IP, PORT):
  """
  First get an image from Nao, then show it on the screen with PIL.
  """

  camProxy = ALProxy("ALVideoDevice", IP, PORT)
  resolution = 2    # VGA
  colorSpace = 11   # RGB

  videoClient = camProxy.subscribe("python_client", resolution, colorSpace, 5)

  t0 = time.time()

  # Get a camera image.
  # image[6] contains the image data passed as an array of ASCII chars.
  naoImage = camProxy.getImageRemote(videoClient)

  t1 = time.time()

  # Time the image transfer.
  print "acquisition delay ", t1 - t0

  camProxy.unsubscribe(videoClient)


  # Now we work with the image returned and save it as a PNG  using ImageDraw
  # package.

  # Get the image size and pixel array.
  imageWidth = naoImage[0]
  imageHeight = naoImage[1]
  array = naoImage[6]

  # Create a PIL Image from our pixel array.
  im = Image.fromstring("RGB", (imageWidth, imageHeight), array)

  # Save the image.
  im.save("camImage.png", "PNG")

  #im.show()

  frame = np.array(im) 
  # Convert RGB to BGR 
  frame = frame[:, :, ::-1].copy()
  hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
  #cv2.startWindowThread()
  #cv2.imshow('image',open_cv_image)
  # define range of blue color in HSV
  #lower_blue = np.array([110,50,50])
  #upper_blue = np.array([130,255,255])
  # create trackbars for color change
  cv2.createTrackbar('Hue low','frame',0,255,nothing)
  hmin = cv2.getTrackbarPos('Hue low','frame')
  cv2.createTrackbar('Hue high','frame',0,255,nothing)
  hmax = cv2.getTrackbarPos('Hue high','frame')
  cv2.createTrackbar('Saturation low','frame',0,255,nothing)
  smin = cv2.getTrackbarPos('Saturation low','frame')
  cv2.createTrackbar('Saturation high','frame',0,255,nothing)
  smax = cv2.getTrackbarPos('Saturation high','frame')
  cv2.createTrackbar('Value low','frame',0,255,nothing)
  vmin = cv2.getTrackbarPos('Value low','frame')
  cv2.createTrackbar('Value high','frame',0,255,nothing)
  vmax = cv2.getTrackbarPos('Value max','frame')

  while(1):
      lower_blue = np.array([hmin,smin,vmin])
      upper_blue = np.array([hmax,smax,vmax])
      
      # Threshold the HSV image to get only blue colors
      mask = cv2.inRange(hsv, lower_blue, upper_blue)
   
      # Bitwise-AND mask and original image
      res = cv2.bitwise_and(frame,frame, mask= mask)

      cv2.imshow('frame',frame)
      cv2.imshow('mask',mask)
      cv2.imshow('res',res)

      k = cv2.waitKey(1) & 0xFF
      if k == 27:
          break
        
  cv2.destroyAllWindows()

  #k = cv2.waitKey(0)
  #if k == 27:         # wait for ESC key to exit
  #    cv2.destroyAllWindows()
  #cv2.waitKey(50)
  #cv2.destroyAllWindows()

##  motionProxy  = ALProxy("ALMotion", IP, PORT)
##  postureProxy = ALProxy("ALRobotPosture", IP, PORT)
##
##  # Wake up robot
##  motionProxy.wakeUp()
##
##  # Send robot to Stand Init
##  postureProxy.goToPosture("StandInit", 0.5)
##
##  # Go to rest position
##  motionProxy.rest()



if __name__ == '__main__':
  IP = "192.168.1.107"  # Replace here with your NaoQi's IP address.
  PORT = 9559

  # Read IP address from first argument if any.
  if len(sys.argv) > 1:
    IP = sys.argv[1]

  naoImage = showNaoImage(IP, PORT)
