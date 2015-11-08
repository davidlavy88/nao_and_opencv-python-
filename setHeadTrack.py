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
scan = qi.PeriodicTask()
motion = ses.service('ALMotion')	
posture = ses.service('ALRobotPosture')
tracker = ses.service('ALTracker')
video = ses.service('ALVideoDevice')

def scan_area():
	# Move neck to extreme right
	motion.setAngles("HeadYaw", math.pi/2, 0.6)
	for


def move_head():
	motion.setPositions('Head', 1, [0,0,0,0,0,0], 0.5, 56)

def move_body():
	motion.moveTo(0,0,1)

def move_both():
	per.start(True)
	qi.async(move_body)

def  main(robotIP, PORT=9559):

    resolution = 2    # VGA
    colorSpace = 11   # RGB

	motion.wakeUp()
	posture.goToPosture('StandInit',0.5)

	#motion.moveTo(0,0,-1)
	#move_head()

	per.setUsPeriod(500000)
	per.setCallback(move_head)
	per.start(True)

	motion.moveTo(0,0,-1)
	#motion.setPositions('Head', 1, [0,0,0,0,0,1.5], 0.5, 56)
	time.sleep(3)

	motion.moveTo(0,0,1)
	#motion.setPositions('Head', 1, [0,0,0,0,0,1.5], 0.5, 56)
	time.sleep(3)

	#motion.moveTo(0,0,-1)
	#motion.setPositions('Head', 1, [0,0,0,0,0,1.5], 0.5, 56)
	#time.sleep(3)

	#move_both()
	#motion.moveTo(0,0,-1)
	#motion.moveTo(0,0,1)

	per.stop()

	#stop move on the next double support
    #motion.stopMove()

    #Go to rest position
   	motion.rest()


if __name__ == "__main__":   
    
    main(robotIP)