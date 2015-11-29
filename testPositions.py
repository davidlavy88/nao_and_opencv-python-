import almath
import argparse
import qi
import math
from functools import partial
import time

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

resolution = 2    # VGA
colorSpace = 11   # RGB
maxAngleScan = math.pi * 2 / 9


def pic(_name):
    videoClient = video.subscribe("python_client", resolution, colorSpace, 5)
    naoImage = video.getImageRemote(videoClient)
    # Get the image size and pixel array.
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]
    im = Image.fromstring("RGB", (imageWidth, imageHeight), str(array))
    im.save(_name, "PNG")


def move_body_back(_angle):
    motion.moveTo(0, 0, _angle)


def move_head():
    motion.angleInterpolationWithSpeed("HeadYaw", 0, 0.1)


def main(robotIP, PORT=9559):
    motion.wakeUp()
    # Testing shown that for every 0.6 angle of rotation add an extra 0.1 to compensate
    # and for 1.3 angle add an extra 0.2
    # posture.goToPosture("StandInit", 1.0)
    motion.moveInit()
    ang = 1.3
    motion.angleInterpolationWithSpeed("Head", [ang, 0.035], 0.1)
    pic("initPic.png")
    time.sleep(0.1)
    head = "Head"
    useSensorValues = True
    result = motion.getRobotPosition(useSensorValues)
    print "Robot Position ", result
    headAngles = motion.getAngles(head, useSensorValues)
    # print "Body Angle", bodyAngles
    print "Head Angle", headAngles
    per.setUsPeriod(500000)
    per.setCallback(move_head)
    per.start(True)
    motion.moveTo(0, 0, ang+0.2)
    per.stop()
    # posture.goToPosture("StandInit", 1.0)
    motion.moveInit()
    pic("finalPic.png")
    time.sleep(0.4)
    motion.rest()


if __name__ == "__main__":
    main(robotIP)
