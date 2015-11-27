import qi
import naoqi
import argparse
import time
import random
import math
from functools import partial

# Python Image Library
import Image

import numpy as np
import cv2

# robotIP = "10.70.122.58"
robotIP = "169.254.252.60"
# robotIP = "192.168.1.107"
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

# motionAngles = []

def CenterOfMass(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img2 = image[:, :, ::-1].copy()

    lowera = np.array([160,100,0])
    uppera = np.array([180,250,255])
    lowerb = np.array([0,100,0])
    upperb = np.array([5,250,255])

    mask1 = cv2.inRange(hsv, lowera, uppera)
    mask2 = cv2.inRange(hsv, lowerb, upperb)

    mask = cv2.add(mask1,mask2)

    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    mj=sum(mask)
    mi=sum(np.transpose(mask))
    A=mask.shape
    ni=np.array(range(A[0]))
    nj=np.array(range(A[1]))
    M=sum(sum(mask))
    if sum(mi)==0 or sum(mj)==0:
        print "no ball"
        xcm=0
        ycm=0
    else:
        xcm=np.dot(mj,nj)/sum(mj)
        ycm=np.dot(mi,ni)/sum(mi)

    CM=[ycm,xcm]

    return CM

def move_head(angleScan):
    print 'moving head'
    angleLists = [[0, angleScan]]
    timeLists = [[1.0, 2.0]]
    motion.angleInterpolation("HeadYaw", angleLists, timeLists, True)

def pic():
    videoClient = video.subscribe("python_client", resolution, colorSpace, 5)    
    naoImage = video.getImageRemote(videoClient)
    # Get the image size and pixel array.
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]
    im = Image.fromstring("RGB", (imageWidth, imageHeight), str(array))
    im.save("ball.png","PNG")

def take_pics(angleScan):
    t = 0
    dt = 1
    s = "camImage"
    ext = ".png"
    n = 0
    names         = "HeadYaw"
    useSensors    = False
    videoClient = video.subscribe("python_client", resolution, colorSpace, 5)
    motionAngles = []
    while t <= 2:
        print 'getting image ' + str(n)
        commandAngles = motion.getAngles(names, useSensors)
        motionAngles.append(commandAngles)
        naoImage = video.getImageRemote(videoClient)
        # Get the image size and pixel array.
        imageWidth = naoImage[0]
        imageHeight = naoImage[1]
        array = naoImage[6]
        im = Image.fromstring("RGB", (imageWidth, imageHeight), str(array))
        name = s+str(n)+ext
        im.save(name, "PNG")
        print 'saved image ' + str(n)
        img = np.array(im)
        # Convert RGB to BGR
        img = img[:, :, ::-1].copy()
        # angle = angle + rotationAngle
        # motion.setAngles("HeadYaw", initAngle, 0.6)
        t = t + dt
        # vec = [t,t,t,t]
        print "Command angles:"
        print str(commandAngles)
        print ""
        n = n + 1
        time.sleep(0.4)
    return motionAngles

def scan_area():
    motion.angleInterpolationWithSpeed( "Head", [-maxAngleScan, 0.035 ], 0.1 );
    
    partial_callback2 = partial(take_pics, maxAngleScan)
    partial_callback = partial(move_head, maxAngleScan)
    fut = qi.async(partial_callback)
    fut2 = qi.async(partial_callback2)
    fut.wait()
    fut2.wait()
    print fut2.value()
    centers = analyze_img()
    # centers = [0, 0]
    return [centers, fut2.value()]

def analyze_img():
    CM = []
    for i in range(1, 4):
        img = cv2.imread("camImage" + str(i-1) + ".png")
        cm = CenterOfMass(img)
        CM.append(cm)
    return CM

# def get_centered_angle(centers, rot_angles):
    # phi = rot_angles[2] - rot_angles[1]
    # d = centers[2] - centers[1]
def rotate_center_head(centers, rot_angles):
    t_ang = (320 - centers[1][1]) * 0.00592
    ang = rot_angles[1][0] + t_ang
    ang = ang.item()
    print type(ang)
    motion.angleInterpolationWithSpeed( "Head", [ang, 0.035 ], 0.1 );

def set_head_position(_angle):
    fracSpeed = 0.2
    names = ['HeadYaw']
    motion.setAngles(names, _angle, fracSpeed)

def  main(robotIP, PORT=9559):

    motion.wakeUp()
    posture.goToPosture('StandInit',0.5)

    [CC, AA] = scan_area()
    print CC
    print AA
    for i in range(3):
        img = cv2.imread("camImage" + str(i) + ".png")
        if CC[i][0] != 0:
            cv2.circle(img,(CC[i][1], CC[i][0]), 2, (0, 255, 0), 3)
            cv2.imshow('detected ball', img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print "No Ball, bitch"

    rotate_center_head(CC,AA)
    pic()
    motion.rest()
    # print commandAngles


if __name__ == "__main__":   
    
    #169.254.252.60
    main(robotIP)
