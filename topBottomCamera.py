import qi
import naoqi
import argparse
import time
import random
import math
from functools import partial
from naoqi import ALProxy

# Python Image Library
import Image

import numpy as np
import cv2

# robotIP = "10.70.122.58"
robotIP = "169.254.252.60"
# robotIP = "192.168.1.107"
# robotIP = "192.168.1.122"
ses = qi.Session()
ses.connect(robotIP)
per = qi.PeriodicTask()
scan = qi.PeriodicTask()
motion = ses.service('ALMotion')
posture = ses.service('ALRobotPosture')
tracker = ses.service('ALTracker')
video = ses.service('ALVideoDevice')
tts = ses.service('ALTextToSpeech')

resolution = 2    # VGA
colorSpace = 11   # RGB
maxAngleScan = math.pi * 2 / 9


def pic(_name, CameraIndex):
    # videoClient = video.subscribe("python_client", resolution, colorSpace, 5)
    videoClient = video.subscribeCamera("python_client", CameraIndex, resolution, colorSpace, 5)
    naoImage = video.getImageRemote(videoClient)
    # Get the image size and pixel array.
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]
    im = Image.fromstring("RGB", (imageWidth, imageHeight), str(array))
    im.save(_name,"PNG")

def main(robotIP, PORT=9559):

    idx = 1
    for i in range(0, 2):
        pp = "test_upfront"
        ext = ".png"
        im_num = pp+str(idx)+ext
        pic(im_num, 0)
        idx = idx + 1
    print "Exiting Top Camera"
    video.stopCamera(0)
    video.startCamera(1)
    video.setActiveCamera(1)
    idx = 1
    for i in range(0, 2):
        pp = "test_downfront"
        ext = ".png"
        im_num = pp+str(idx)+ext
        pic(im_num, 1)
        idx = idx + 1
    print "Exiting Bottom Camera"


if __name__ == "__main__":

    main(robotIP)