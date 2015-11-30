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

# motionAngles = []
def analyze_img():
    CM = []
    for i in range(0, 5):
        img = cv2.imread("camImage" + str(i) + ".png")
        cm = CenterOfMass(img, 0)
        CM.append(cm)
    return CM

def CenterOfMass(image, CameraIndex):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img2 = image[:, :, ::-1].copy()

    if CameraIndex == 0:
        lowera = np.array([160, 165, 0])
        uppera = np.array([180, 250, 255])
        lowerb = np.array([0, 165, 0])  # It was 100 instead of 195
        upperb = np.array([5, 250, 255])
    elif CameraIndex == 1:
        lowera = np.array([160, 95, 0])
        uppera = np.array([180, 250, 255])
        lowerb = np.array([0, 95, 0])  # It was 100 instead of 195
        upperb = np.array([10, 250, 255])

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

def numBalls(CM):
    """Takes in the CM list, outputs indices of frames containing balls"""
    index=[]
    for i in range(len(CM)):
        if CM[i]!=[0,0]:
            index.append(i)
    return index

def pic(_name, CameraIndex):
    # videoClient = video.subscribe("python_client", resolution, colorSpace, 5)
    videoClient = video.subscribeCamera("python_client", CameraIndex, resolution, colorSpace, 5)
    naoImage = video.getImageRemote(videoClient)
    video.unsubscribe(videoClient)
    # Get the image size and pixel array.
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]
    im = Image.fromstring("RGB", (imageWidth, imageHeight), str(array))
    im.save(_name, "PNG")

def rotate_center_head(centers, rot_angles):
    index=numBalls(centers)
    adj = 60
    if len(index)==0:
        string="I don't see the ball."
        ang = 0
        state = 0
        RF = 0
    elif len(index)==1:
        a=index[0]
        string="I need to get a better look at the ball."
        ang=rot_angles[a][0]
        # ang = ang.item()
        state=1
        RF = 0
    else:
        string="I see the ball."
        a=index[0]
        b=index[1]
        RF=(rot_angles[b][0]-rot_angles[a][0])/(centers[a][1]-centers[b][1])
        ang = rot_angles[a][0] - (320- adj - centers[a][1])*RF
        ang = ang.item()
        state=2
    print ang
    motion.angleInterpolationWithSpeed( "Head", [ang, 0.035 ], 0.1 );
    tts.say(string)
    return [ang,state,RF]


def scan_area():
    motion.angleInterpolationWithSpeed("Head", [-maxAngleScan, 0.035], 0.1)
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


def set_head_position(_angle):
    fracSpeed = 0.2
    names = ['HeadYaw']
    motion.setAngles(names, _angle, fracSpeed)


def show_CM(_name, CameraIndex):
    img = cv2.imread(_name)
    CM = CenterOfMass(img, CameraIndex)
    # print CM
    # cv2.circle(img, (CM[1], CM[0]), 2, (0, 255, 0), 3)
    # cv2.imshow('detected ball', img)
    # cv2.imshow(_name, img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return CM


def take_pics(angleScan):
    t = 0
    dt = 1
    s = "camImage"
    ext = ".png"
    n = 0
    names = "HeadYaw"
    useSensors = False
    videoClient = video.subscribe("python_client", resolution, colorSpace, 5)
    motionAngles = []
    while t <= 4:
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
        time.sleep(0.1)
    return motionAngles


def zero_head():
    motion.angleInterpolationWithSpeed("HeadYaw", 0, 0.1)


def main(robotIP, PORT=9559):

    motion.wakeUp()
    # posture.goToPosture('StandInit', 0.5)
    motion.moveInit()
    X = 0
    cm = []
    while X == 0:
        [CC, AA] = scan_area()
#############DEBUG#VERSION###############
##    indices=numBalls(CC)
##    
##    for i in indices:
##        img = cv2.imread("camImage" + str(i) + ".png")
##        cv2.circle(img,(CC[i][1], CC[i][0]), 2, (0, 255, 0), 3)
##        cv2.imshow('detected ball', img)
##        cv2.waitKey(0)
##        cv2.destroyAllWindows()
#########################################
        [ang,X,delta] = rotate_center_head(CC,AA)
        if X==1: # turn to angle of pic with ball and scan
           motion.moveTo(0, 0, ang)
           X=0
        elif X==0: # didn't see a ball, rotate 80 degrees and scan
           motion.moveTo(0, 0, 80 * math.pi/180)
    
    pic("ball.png",0)
    cm = show_CM("ball.png", 0)
    print cm

    per.setUsPeriod(500000)
    per.setCallback(zero_head)
    per.start(True)
    motion.moveTo(0, 0, ang*7/6)
    per.stop()
    pic("ball_upfront.png",0)
    cm = show_CM("ball_upfront.png", 0)
    print cm

    idx = 1
    print "Entering the loop"
    while cm[0] <  420:
        pp = "ball_upfront"
        ext = ".png"
        motion.moveTo(0.2, 0, 0)
        im_num = pp+str(idx)+ext
        pic(im_num,0)
        cm = show_CM(im_num, 0)
        print cm
        alpha = (cm[1] - 320) * delta
        alpha = alpha.item()
        print "Angle to rotate", alpha
        motion.angleInterpolationWithSpeed("HeadYaw", alpha, 0.1)
        zero_head()
        motion.moveTo(0, 0, alpha*7/6)
        idx = idx + 1
        # cv2.destroyAllWindows()
    print "Exiting up loop"
    motion.moveTo(0.15, 0, 0)
    # im_num = pp+str(idx)+ext
    # pic(im_num)
    # cm = show_CM(im_num)
    # print cm
    # alpha = (cm[1] - 320) * delta
    # alpha = alpha.item()
    # print "Angle to rotate ", alpha
    # motion.angleInterpolationWithSpeed("HeadYaw", alpha, 0.1)
    # zero_head()
    time.sleep(0.2)
    video.stopCamera(0)
    video.startCamera(1)
    video.setActiveCamera(1)
    pic("bottom.png",1)
    cm = show_CM("bottom.png", 1)
    print cm
    alpha = (cm[1] - 320) * delta
    alpha = alpha.item()
    print "Angle to rotate ", alpha
    motion.angleInterpolationWithSpeed("HeadYaw", alpha, 0.1)
    zero_head()
    motion.moveTo(0, 0, alpha*7/6)
    idx = 1
    print "Entering the loop"
    while cm[0] <  420:
        pp = "ball_downfront"
        ext = ".png"
        motion.moveTo(0.2, 0, 0)
        im_num = pp+str(idx)+ext
        pic(im_num, 1)
        cm = show_CM(im_num, 1)
        print cm
        alpha = (cm[1] - 320) * delta
        alpha = alpha.item()
        print "Angle to rotate ", alpha
        motion.angleInterpolationWithSpeed("HeadYaw", alpha, 0.1)
        zero_head()
        motion.moveTo(0, 0, alpha*7/6)
        idx = idx + 1
        print cm
    motion.moveTo(0.15, 0, 0)
    im_num = pp+str(idx)+ext
    print "I'm here"
    tts.say("Almost there!")
    pic(im_num,1)
    cm = show_CM(im_num, 1)
    print cm
    tts.say("I'm in front of the ball")


    # motion.moveTo(0.2, 0, 0)
    # pic("ball_upfront1.png")
    # cm = show_CM("ball_upfront1.png")
    # # get angle of rotation
    # alpha = (cm[1] - 320) * delta
    # alpha = alpha.item()
    # motion.angleInterpolationWithSpeed("HeadYaw", alpha, 0.1)
    # pic("ball_upfront2.png")
    # cm = show_CM("ball_upfront2.png")
    # print cm

    motion.rest()
    # print commandAngles


if __name__ == "__main__":   
    
    #169.254.252.60
    main(robotIP)
