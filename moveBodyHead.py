# -*- encoding: UTF-8 -*-

import argparse 
import time
import random
import math
from naoqi import ALProxy

# Python Image Library
import Image

import numpy as np
import cv2

def main(robotIP, PORT=9559):
    
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    
    camProxy = ALProxy("ALVideoDevice", robotIP, PORT)
    resolution = 2    # VGA
    colorSpace = 11   # RGB

    videoClient = camProxy.subscribe("python_client", resolution, colorSpace, 5)
    
    # Wake up robot
    motionProxy.wakeUp()
    
    # Send robot to Stand Init
    postureProxy.goToPosture("StandInit", 0.5)
    
    motionProxy.moveInit()
    
    # Move Head to the left
    motionProxy.setAngles("HeadYaw", -math.pi/4.0, 0.6)
    
    time.sleep(1)
    
    t0 = time.time()
    
    # Get a camera image.
    # image[6] contains the image data passed as an array of ASCII chars.
    naoImage = camProxy.getImageRemote(videoClient)

    t1 = time.time()

    # Time the image transfer.
    print "acquisition delay ", t1 - t0

    #camProxy.unsubscribe(videoClient)
    
    # Get the image size and pixel array.
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]
    
    # Create a PIL Image from our pixel array.
    im = Image.fromstring("RGB", (imageWidth, imageHeight), array)
    
    # Save the image.
    im.save("./Images/camBefore.png", "PNG")
    
    im.show()
    
    # Get position of the head to set it fixed
    names         = "HeadYaw"
    useSensors    = False
    commandAngles = motionProxy.getAngles(names, useSensors)
    
    print "Command angles:"
    print str(commandAngles)
    print ""
    
    testTime = 1.0 # seconds
    t  = 0.0
    dt = 0.2
    
##    while t<testTime:
##        # WALK
##        X         = 0
##        Y         = 0
##        Theta     = math.pi/4.0
##        #Frequency = random.uniform(0.5, 1.0)
##        try:
##            #motionProxy.moveToward(X, Y, Theta, [["Frequency", Frequency]])
##            motionProxy.move(0, 0, Theta)
##            #motionProxy.setAngles("HeadYaw", -0.5, 0.6)
##        except Exception, errorMsg:
##            print str(errorMsg)
##            print "This example is not allowed on this robot."
##            exit()
##
##        # JERKY HEAD
##        #motionProxy.setAngles("HeadYaw", -0.5, 0.6)
##        #motionProxy.setAngles("HeadPitch", random.uniform(-0.5, 0.5), 0.6)
##
##        t = t + dt
##        time.sleep(dt)

    X         = 0
    Y         = 0
    Theta     = math.pi/4.0

    motionProxy.moveTo(X, Y, Theta)
    
    # Rotate the head the displacement of the body
    motionProxy.setAngles("HeadYaw", -math.pi/2.0, 0.6)
    
    time.sleep(1)
    
    t0 = time.time()
    
    # Get a camera image.
    # image[6] contains the image data passed as an array of ASCII chars.
    naoImage = camProxy.getImageRemote(videoClient)

    t1 = time.time()

    # Time the image transfer.
    print "acquisition delay ", t1 - t0
    
    # Get the image size and pixel array.
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]
    
    # Create a PIL Image from our pixel array.
    im = Image.fromstring("RGB", (imageWidth, imageHeight), array)
    
    # Save the image.
    im.save("./Images/camAfter.png", "PNG")
    
    im.show()
    
    time.sleep(0.5)
    
    # stop move on the next double support
    motionProxy.stopMove()

    # Go to rest position
    motionProxy.rest()
    
if __name__ == "__main__":
    robotIP = "192.168.1.107"
    
    main(robotIP)