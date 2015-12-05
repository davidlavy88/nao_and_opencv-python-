import qi
import argparse
import time
import math
from functools import partial
import numpy as np
import motion as mot

# Python Image Library
import Image
# OpenCV Libraries
import cv2

''' ROBOT CONFIGURATION '''
# robotIP = "10.70.122.58"
# robotIP = "169.254.252.60"
# robotIP = "192.168.1.107"
robotIP = "192.168.1.122"
ses = qi.Session()
ses.connect(robotIP)
per = qi.PeriodicTask()
motion = ses.service('ALMotion')
posture = ses.service('ALRobotPosture')
tracker = ses.service('ALTracker')
video = ses.service('ALVideoDevice')
tts = ses.service('ALTextToSpeech')
landmark = ses.service('ALLandMarkDetection')
memory = ses.service('ALMemory')

resolution = 2    # VGA
colorSpace = 11   # RGB


# During the initial scan, take a few pictures to analize where's the ball
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
    arrayIm = []
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
        arrayIm.append(im)
        ##############################
        ''' This part will be deleted and we just need to return a array of all
        the ims in the output '''
        name = s+str(n)+ext
        im.save(name, "PNG")
        print 'saved image ' + str(n)
        #############################
        t = t + dt
        print "Command angles:"
        print str(commandAngles)
        print ""
        n = n + 1
        time.sleep(0.1)
    video.unsubscribe(videoClient)
    # return motionAngles, arrayIm
    return motionAngles


# Move HeadYaw from [-angleScan;angleScan]
def move_head(angleScan):
    print 'moving head'
    angleLists = [[0, angleScan]]
    timeLists = [[1.0, 2.0]]
    motion.angleInterpolation("HeadYaw", angleLists, timeLists, True)


# Calculate CoM of the thresholded ball (center of the circle)
# TODO: Calculate the shape of the ball so CoM would be more accurate
def CenterOfMass(image, CameraIndex):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # img2 = image[:, :, ::-1].copy()

    if CameraIndex == 0:
        lowera = np.array([160, 165, 0])
        uppera = np.array([180, 250, 255])
        lowerb = np.array([0, 165, 0])  # It was 100 instead of 195
        upperb = np.array([5, 250, 255])
    elif CameraIndex == 1:
        # lowera = np.array([160, 95, 0])
        # uppera = np.array([180, 250, 255])
        # lowerb = np.array([0, 95, 0])  # It was 100 instead of 195
        # upperb = np.array([10, 250, 255])
        lowera = np.array([160, 165, 0])
        uppera = np.array([180, 250, 255])
        lowerb = np.array([0, 165, 0])  # It was 100 instead of 195
        upperb = np.array([10, 250, 255])

    mask1 = cv2.inRange(hsv, lowera, uppera)
    mask2 = cv2.inRange(hsv, lowerb, upperb)

    mask = cv2.add(mask1, mask2)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    mj = sum(mask)
    mi = sum(np.transpose(mask))
    A = mask.shape
    ni = np.array(range(A[0]))
    nj = np.array(range(A[1]))
    # M = sum(sum(mask))
    if sum(mi) == 0 or sum(mj) == 0:
        print "no ball"
        xcm = 0
        ycm = 0
    else:
        xcm = np.dot(mj, nj)/sum(mj)
        ycm = np.dot(mi, ni)/sum(mi)

    CM = [ycm, xcm]
    return CM


# Find the center of mass of the ball
def analyze_img():
    CM = []
    for i in range(0, 5):
        img = cv2.imread("camImage" + str(i) + ".png")
        cm = CenterOfMass(img, 0)
        CM.append(cm)
    return CM


# Look if the ball is in front of the robot
def scan_area(_angleSearch):
    # Search angle where angle of rotation is [-maxAngleScan;+maxAngleScan]
    maxAngleScan = _angleSearch
    motion.angleInterpolationWithSpeed("Head", [-maxAngleScan, 0.035], 0.1)
    partial_callback2 = partial(take_pics, maxAngleScan)
    partial_callback = partial(move_head, maxAngleScan)
    fut = qi.async(partial_callback)
    fut2 = qi.async(partial_callback2)
    fut.wait()
    fut2.wait()
    print fut2.value()
    centers = analyze_img()
    return [centers, fut2.value()]


# Index of pictures that contains the ball
def numBalls(CM):
    """Takes in the CM list, outputs indices of frames containing balls"""
    index = []
    for i in range(len(CM)):
        if CM[i] != [0, 0]:
            index.append(i)
    return index


# Find the ball and center its look to it, otherwise back to 0 and rotate again
def rotate_center_head(centers, rot_angles):
    index = numBalls(centers)
    adj = 60
    if len(index) == 0:
        string = "I don't see the ball."
        ang = 0
        state = 0
        RF = 0
    elif len(index) == 1:
        a = index[0]
        string = "I need to get a better look at the ball."
        ang = rot_angles[a][0]
        # ang = ang.item()
        state = 1
        RF = 0
    else:
        string = "I see the ball."
        a = index[0]
        b = index[1]
        RF = (rot_angles[b][0] - rot_angles[a][0]) / (centers[a][1] - centers[b][1])
        ang = rot_angles[a][0] - (320 - adj - centers[a][1])*RF
        ang = ang.item()
        state = 2
    print ang
    motion.angleInterpolationWithSpeed("Head", [ang, 0.035], 0.1)
    tts.say(string)
    return [ang, state, RF]


# Will take 1 picture (and return it)
# def pic(CameraIndex):
def pic(_name, CameraIndex):
    # videoClient = video.subscribe("python_client", resolution, colorSpace, 5)
    videoClient = video.subscribeCamera(
        "python_client", CameraIndex, resolution, colorSpace, 5)
    naoImage = video.getImageRemote(videoClient)
    video.unsubscribe(videoClient)
    # Get the image size and pixel array.
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]
    im = Image.fromstring("RGB", (imageWidth, imageHeight), str(array))
    im.save(_name, "PNG")  # Line marked for deletion
    # return im


####################################
''' This function will be deleted as it is only needed to
call CenterOfMass. show_CM is just for visualization '''
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
##################################


# Funtion that will look for position of the ball and return it
def initial_scan():
    angleSearch = math.pi * 2 / 9
    # Get ready to move
    motion.moveInit()
    X = 0
    while X == 0:
        [CC, AA] = scan_area(angleSearch)
        [ang, X, delta] = rotate_center_head(CC, AA)
        # TODO: ang should be constrained to make sure false balls
        # are not encountered
        if ang > angleSearch:
            X = 0
            continue
        if X == 1:  # turn to angle of pic with ball and scan
            motion.moveTo(0, 0, ang)
            X = 0
        elif X == 0:  # didn't see a ball, rotate 80 degrees and scan
            motion.moveTo(0, 0, 80 * math.pi/180)
    pic("ball.png", 0)
    ballPos = show_CM("ball.png", 0)
    print ballPos
    per.setUsPeriod(500000)
    per.setCallback(zero_head)
    per.start(True)
    motion.moveTo(0, 0, ang*7/6)
    per.stop()
    pic("ball_upfront.png", 0)
    ballPos = show_CM("ball_upfront.png", 0)
    print ballPos
    return ballPos, delta


def walkUp(cm, delta):
    idx = 1
    lowerFlag = 0
    print "Entering uppercam loop"
    while cm[0] < 420 and cm[0] > 0:
        pp = "ball_upfront"
        ext = ".png"
        motion.moveTo(0.2, 0, 0)
        im_num = pp+str(idx)+ext
        pic(im_num, 0)
        cm = show_CM(im_num, 0)
        print cm
        if cm[0] == 0 and cm[1] == 0:
            # Scan the area with lower camera
            pic('lower.png', 1)
            cm2 = show_CM('lower.png', 1)
            lowerFlag = 1
            break
        else:
            alpha = (cm[1] - 320) * delta
            alpha = alpha.item()
            # print "Angle to rotate", alpha
            # motion.angleInterpolationWithSpeed("HeadYaw", alpha, 0.1)
            # zero_head()
            motion.moveTo(0, 0, alpha*7/6)
            idx = idx + 1
            # cv2.destroyAllWindows()
            continue
    if lowerFlag == 1:
        if cm2[0] == 0 and cm2[1] == 0:
            lostFlag = 1
            print 'I lost the ball'
        else:
            lostFlag = 0
            print 'I need to switch cameras'
    else:
        pic('lower.png', 1)
        cm2 = show_CM('lower.png', 1)
        lostFlag = 0
    print "Exiting up loop"
    return lostFlag, cm2
    # motion.moveTo(0.15, 0, 0)


def walkDown(cm, delta):
    # Do the correction before it starts the loop
    # alpha = (cm[1] - 320) * delta
    # alpha = alpha.item()
    # motion.moveTo(0, 0, alpha*7/6)
    idx = 1
    pp = "ball_downfront"
    ext = ".png"
    print 'Entering lowercam loop'
    while cm[0] > 0 and cm[0] < 230:
        motion.moveTo(0.2, 0, 0)
        im_num = pp+str(idx)+ext
        pic(im_num, 1)
        cm = show_CM(im_num, 1)
        print im_num, cm
        alpha = (cm[1] - 320) * delta
        alpha = alpha.item()
        motion.moveTo(0, 0, alpha*7/6)
        idx = idx + 1
    # Tilt the head so it can have a better look of the ball
    anglePitch = math.pi * 20.6 / 180
    motion.angleInterpolationWithSpeed("HeadPitch", anglePitch, 0.1)
    while cm[0] >= 30  and cm[0] < 400:
        motion.moveTo(0.2, 0, 0)
        im_num = pp+str(idx)+ext
        pic(im_num, 1)
        cm = show_CM(im_num, 1)
        print im_num, cm
        alpha = (cm[1] - 320) * delta
        alpha = alpha.item()
        motion.moveTo(0, 0, alpha*8/6)
        idx = idx + 1


def kickBall():
    # Activate Whole Body Balancer
    isEnabled  = True
    motion.wbEnable(isEnabled)

    # Legs are constrained fixed
    stateName  = "Fixed"
    supportLeg = "Legs"
    motion.wbFootState(stateName, supportLeg)

    # Constraint Balance Motion
    isEnable   = True
    supportLeg = "Legs"
    motion.wbEnableBalanceConstraint(isEnable, supportLeg)

    # Com go to LLeg
    supportLeg = "LLeg"
    duration   = 1.0
    motion.wbGoToBalance(supportLeg, duration)

    # RLeg is free
    stateName  = "Free"
    supportLeg = "RLeg"
    motion.wbFootState(stateName, supportLeg)

    # RLeg is optimized
    effectorName = "RLeg"
    axisMask     = 63
    # space        = motion.FRAME_ROBOT
    space        = mot.FRAME_TORSO
    # space        = motion.FRAME_WORLD


    # Motion of the RLeg
    dx      = 0.025                 # translation axis X (meters)
    dz      = 0.02                 # translation axis Z (meters)
    dwy     = 5.0*math.pi/180.0    # rotation axis Y (radian)


    times   = [1.0, 1.4, 2.1]
    isAbsolute = False

    targetList = [
      [-0.7*dx, 0.0, dz, 0.0, +dwy, 0.0],
      [+2.2*dx, +dx, dz, 0.0, -dwy, 0.0],
      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

    motion.positionInterpolation(effectorName, space, targetList,
                                 axisMask, times, isAbsolute)


    # Example showing how to Enable Effector Control as an Optimization
    isActive     = False
    motion.wbEnableEffectorOptimization(effectorName, isActive)

    # # Com go to LLeg
    # supportLeg = "RLeg"
    # duration   = 2.0
    # motion.wbGoToBalance(supportLeg, duration)

    # # RLeg is free
    # stateName  = "Free"
    # supportLeg = "LLeg"
    # motion.wbFootState(stateName, supportLeg)

    # effectorName = "LLeg"
    # motion.positionInterpolation(effectorName, space, targetList,
    #                             axisMask, times, isAbsolute)

    time.sleep(1.0)

    # Deactivate Head tracking
    isEnabled    = False
    motion.wbEnable(isEnabled)

    # send robot to Pose Init
    posture.goToPosture("StandInit", 0.5)


def set_head_position(_angle):
    fracSpeed = 0.2
    names = ['HeadYaw']
    motion.setAngles(names, _angle, fracSpeed)


def zero_head():
    motion.angleInterpolationWithSpeed("HeadYaw", 0, 0.1)


def lookForLandmark():
    period = 500
    landmark.subscribe('Test_mark', period, 0.0)
    memValue = 'LandmarkDetected'
    landmarkFound = []
    foundFlag = []
    for i in range(0, 10):
        time.sleep(0.2)
        val = memory.getData(memValue, 0)
        print ""
        print "\*****"
        print ""
        # Check whether we got a valid output: a list with two fields.
        if(val and isinstance(val, list) and len(val) >= 2):
            # We detected landmarks !
            # For each mark, we can read its shape info and ID.
            # Second Field = array of Mark_Info's.
            markInfoArray = val[1]
            for markInfo in markInfoArray:
                # First Field = Shape info.
                markShapeInfo = markInfo[0]
                # Print Mark information.
                angle = float(int(1000*markShapeInfo[1]))/1000
                print 'Angle to rotate', angle
                landmarkFound.append(angle)
    if len(landmarkFound) == 0:
        foundFlag = 0
        rotAngle = 0
    else:
        foundFlag = 1
        rotAngle = landmarkFound[0]
    # Unsubscribe from the module.
    landmark.unsubscribe("Test_mark")
    return foundFlag, rotAngle


def findGoal():
    # Set head to zero position
    motion.angleInterpolationWithSpeed(["HeadYaw", "HeadPitch"], [0, 0], 0.1)
    # Start scan, per 30 degrees of rotation
    


def main(robotIP, PORT=9559):
    # Wake Up the robot
    motion.wakeUp()
    ballPosition, delta = initial_scan()
    # Walk to the ball using upper camera
    lost, CoM = walkUp(ballPosition, delta)
    if lost == 1:
        tts.say('I lost the ball, I need to re-scan the room')
        done = 'Could not find the ball'
        # return done
    else:
        # Switch cameras
        time.sleep(0.2) 
        video.stopCamera(0)
        video.startCamera(1)
        video.setActiveCamera(1)
        # Walk to the ball using lower camera
        walkDown(CoM, delta)


    # Kick the ball
    kickBall()


    ###### THIS WILL BE REPLACED #####
#     # Get ready to move
#     motion.moveInit()
#     X = 0
#     cm = []
#     while X == 0:
#         [CC, AA] = scan_area()
# #############DEBUG#VERSION###############
# ##    indices=numBalls(CC)
# ##    
# ##    for i in indices:
# ##        img = cv2.imread("camImage" + str(i) + ".png")
# ##        cv2.circle(img,(CC[i][1], CC[i][0]), 2, (0, 255, 0), 3)
# ##        cv2.imshow('detected ball', img)
# ##        cv2.waitKey(0)
# ##        cv2.destroyAllWindows()
# #########################################
#         [ang,X,delta] = rotate_center_head(CC,AA)
#         if X==1: # turn to angle of pic with ball and scan
#            motion.moveTo(0, 0, ang)
#            X=0
#         elif X==0: # didn't see a ball, rotate 80 degrees and scan
#            motion.moveTo(0, 0, 80 * math.pi/180)
    
#     pic("ball.png",0)
#     cm = show_CM("ball.png", 0)
#     print cm
    ################################

    ##########################################
    # per.setUsPeriod(500000)
    # per.setCallback(zero_head)
    # per.start(True)
    # motion.moveTo(0, 0, ang*7/6)
    # per.stop()
    # pic("ball_upfront.png",0)
    # cm = show_CM("ball_upfront.png", 0)
    # print cm



    # idx = 1
    # print "Entering the loop"
    # while cm[0] <  420:
    #     pp = "ball_upfront"
    #     ext = ".png"
    #     motion.moveTo(0.2, 0, 0)
    #     im_num = pp+str(idx)+ext
    #     pic(im_num,0)
    #     cm = show_CM(im_num, 0)
    #     print cm
    #     alpha = (cm[1] - 320) * delta
    #     alpha = alpha.item()
    #     print "Angle to rotate", alpha
    #     motion.angleInterpolationWithSpeed("HeadYaw", alpha, 0.1)
    #     zero_head()
    #     motion.moveTo(0, 0, alpha*7/6)
    #     idx = idx + 1
    #     # cv2.destroyAllWindows()
    # print "Exiting up loop"
    # motion.moveTo(0.15, 0, 0)



    # im_num = pp+str(idx)+ext
    # pic(im_num)
    # cm = show_CM(im_num)
    # print cm
    # alpha = (cm[1] - 320) * delta
    # alpha = alpha.item()
    # print "Angle to rotate ", alpha
    # motion.angleInterpolationWithSpeed("HeadYaw", alpha, 0.1)
    # zero_head()
    # time.sleep(0.2)
    # video.stopCamera(0)
    # video.startCamera(1)
    # video.setActiveCamera(1)



    # pic("bottom.png",1)
    # cm = show_CM("bottom.png", 1)
    # print cm
    # alpha = (cm[1] - 320) * delta
    # alpha = alpha.item()
    # print "Angle to rotate ", alpha
    # motion.angleInterpolationWithSpeed("HeadYaw", alpha, 0.1)
    # zero_head()
    # motion.moveTo(0, 0, alpha*7/6)
    # idx = 1
    # print "Entering the loop"
    # while cm[0] <  420:
    #     pp = "ball_downfront"
    #     ext = ".png"
    #     motion.moveTo(0.2, 0, 0)
    #     im_num = pp+str(idx)+ext
    #     pic(im_num, 1)
    #     cm = show_CM(im_num, 1)
    #     print cm
    #     alpha = (cm[1] - 320) * delta
    #     alpha = alpha.item()
    #     print "Angle to rotate ", alpha
    #     motion.angleInterpolationWithSpeed("HeadYaw", alpha, 0.1)
    #     zero_head()
    #     motion.moveTo(0, 0, alpha*7/6)
    #     idx = idx + 1
    #     print cm
    # motion.moveTo(0.15, 0, 0)
    # im_num = pp+str(idx)+ext
    # print "I'm here"
    # tts.say("Almost there!")
    # pic(im_num,1)
    # cm = show_CM(im_num, 1)
    # print cm
    # tts.say("I'm in front of the ball")
    ######################################


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
