import qi
import naoqi
import argparse 
import time
import random
import math

# Python Image Library
import Image

import numpy as np
import cv2

robotIP = "192.168.1.107"
ses = qi.Session()
ses.connect(robotIP)
per = qi.PeriodicTask()
motion = ses.service('ALMotion')	
posture = ses.service('ALRobotPosture')

def move_head():
	motion.setPositions('Head', 1, [0,0,0,0,0,0], 0.5, 56)

def move_body():
	motion.moveTo(0,0,1)

def move_both():
	per.start(True)
	take_picture_per.start(True)
	move_body()
	per.stop()

def  main(robotIP, PORT=9559):
	motion.wakeUp()
	posture.goToPosture('StandInit',0.5)

	#motion.moveTo(0,0,-1)
	move_head()

	per.setUsPeriod(500000)
	per.setCallback(move_head)

	move_both()
	#motion.moveTo(0,0,-1)
	#motion.moveTo(0,0,1)

	per.stop()

	#stop move on the next double support
    #motion.stopMove()

    #Go to rest position
   	motion.rest()


if __name__ == "__main__":   
    
    main(robotIP)
