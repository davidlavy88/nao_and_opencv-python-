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
    trackerProxy = ALProxy("ALTracker", robotIP, PORT)
    
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
    #motionProxy.setAngles("HeadYaw", -math.pi/4.0, 0.6)
    trackerProxy.lookAt([-5, 5, 1], 1, 0.2, False)
    
    time.sleep(1)

    
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
    Theta     = -math.pi/4.0

    motionProxy.moveTo(X, Y, Theta)
    
    # Rotate the head the displacement of the body
    #motionProxy.setAngles("HeadYaw", -math.pi/2.0, 0.6)
    trackerProxy.lookAt([-5, 5, 1], 1, 0.2, False)
    
    time.sleep(1)
    
    
    # stop move on the next double support
    motionProxy.stopMove()

    # Go to rest position
    motionProxy.rest()
    
if __name__ == "__main__":
    robotIP = "192.168.1.107"
    #robotIP = "10.1.42.50"
    
    main(robotIP)
