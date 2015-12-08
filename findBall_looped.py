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
import cv2.cv as cv

''' ROBOT CONFIGURATION '''
# robotIP = "10.70.122.53"
# robotIP = "169.254.252.60"
robotIP = "169.254.194.108"
# robotIP = "192.168.1.107"
# robotIP = "192.168.1.122"
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
trial_number = 10
path = 'trials/trial' + str(trial_number) + '/'


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
        name = path + s+str(n)+ext
        # name = s+str(n)+ext
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
def CenterOfMassUp(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # 100 lower range of saturation for daveed's place, day. 165 night 95 sat; 10 hue bottom
    # 195 lower range of saturation for travis' place
    lowera = np.array([160, 95, 0])
    uppera = np.array([180, 250, 255])
    lowerb = np.array([0, 95, 0])
    upperb = np.array([10, 250, 255])

    mask1 = cv2.inRange(hsv, lowera, uppera)
    mask2 = cv2.inRange(hsv, lowerb, upperb)
    mask = cv2.add(mask1,mask2)
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    cont, hier = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    icenter = []
    if len(cont)>=1:
        maxim=0
        for i in range(len(cont)):
            if len(cont[i])>len(cont[maxim]):
                maxim=i
        contour=cont[maxim]
        center, radius = cv2.minEnclosingCircle(contour)
        icenter.append(int(center[0]))
        icenter.append(int(center[1]))
        radius = int(radius)
        print 'Center', center, '. Radius', radius
    
        maxi=np.amax(contour,0)
        mini=np.amin(contour,0)
        center=(maxi+mini)/2
        d = radius
        err=[]
        for i in range(len(contour)):
            t=[contour[i,0,0]-center[0,0],contour[i,0,1]-center[0,1]]
            t=math.sqrt(math.pow(t[0],2)+math.pow(t[1],2))
            e=abs(d-t)/d
            err=err+[e]
        ERR=np.mean(err)
        print "Error value: ", ERR

        if ERR>.22:
            i=0
            j=0
        elif radius>=8 and radius<60:
            i=icenter[1]
            j=icenter[0]
        else:
            i=0
            j=0
    else:
        i=0
        j=0
        contour=cont

    CM=[i,j]
    # remove output of contour for use in code
    return CM


def CenterOfMassDown(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    #100 lower range of saturation for daveed's place, day. 165 night 95 sat; 10 hue bottom
    #195 lower range of saturation for travis' place
    kernel = np.ones((5,5),np.uint8)
    lowera = np.array([160,95,0])
    uppera = np.array([180,250,255])
    lowerb = np.array([0,95,0])
    upperb = np.array([10,250,255])

    mask1 = cv2.inRange(hsv, lowera, uppera)
    mask2 = cv2.inRange(hsv, lowerb, upperb)
    mask = cv2.add(mask1,mask2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    cont, hier = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    icenter = []
    if len(cont)>=1:
        maxim=0
        for i in range(len(cont)):
            if len(cont[i])>len(cont[maxim]):
                maxim=i
        contour=cont[maxim]
        
        center, radius = cv2.minEnclosingCircle(contour)
        icenter.append(int(center[0]))
        icenter.append(int(center[1]))
        radius = int(radius)
        print 'Center', icenter, '. Radius', radius    
##        maxi=np.amax(contour,0)
##        mini=np.amin(contour,0)
##        center=(maxi+mini)/2
        if radius>8 and radius<60:
            i=icenter[1]
            j=icenter[0]
        else:
            i=0
            j=0

    else:
        i=0
        j=0
        contour=cont

    CM=[i,j]
    # remove output of contour for use in code
    return CM


# Find the center of mass of the ball
def analyze_img():
    CM = []
    for i in range(0, 7):
        img = cv2.imread(path + "camImage" + str(i) + ".png")
        cm = CenterOfMassUp(img)
        CM.append(cm)
    return CM


# Look if the ball is in front of the robot
def scan_area(_angleSearch, CameraIndex):
    # Search angle where angle of rotation is [-maxAngleScan;+maxAngleScan]
    names = "HeadYaw"
    useSensors = False
    motionAngles = []
    maxAngleScan = _angleSearch

    motion.angleInterpolationWithSpeed("Head", [-maxAngleScan, 0.035], 0.1)
    pic(path + 'camImage0.png', CameraIndex)
    commandAngles = motion.getAngles(names, useSensors)
    motionAngles.append(commandAngles)
    print str(commandAngles)
    motion.angleInterpolationWithSpeed("Head", [-2*maxAngleScan/3, 0.035], 0.1)
    pic(path + 'camImage1.png', CameraIndex)
    commandAngles = motion.getAngles(names, useSensors)
    motionAngles.append(commandAngles)
    print str(commandAngles)
    motion.angleInterpolationWithSpeed("Head", [-maxAngleScan/3, 0.035], 0.1)
    pic(path + 'camImage2.png', CameraIndex)
    commandAngles = motion.getAngles(names, useSensors)
    motionAngles.append(commandAngles)
    print str(commandAngles)
    motion.angleInterpolationWithSpeed("Head", [0, 0.035], 0.1)
    pic(path + 'camImage3.png', CameraIndex)
    commandAngles = motion.getAngles(names, useSensors)
    motionAngles.append(commandAngles)
    print str(commandAngles)
    motion.angleInterpolationWithSpeed("Head", [maxAngleScan/3, 0.035], 0.1)
    pic(path + 'camImage4.png', CameraIndex)
    commandAngles = motion.getAngles(names, useSensors)
    motionAngles.append(commandAngles)
    print str(commandAngles)
    motion.angleInterpolationWithSpeed("Head", [2*maxAngleScan/3, 0.035], 0.1)
    pic(path + 'camImage5.png', CameraIndex)
    commandAngles = motion.getAngles(names, useSensors)
    motionAngles.append(commandAngles)
    print str(commandAngles)
    motion.angleInterpolationWithSpeed("Head", [maxAngleScan, 0.035], 0.1)
    pic(path + 'camImage6.png', CameraIndex)
    commandAngles = motion.getAngles(names, useSensors)
    motionAngles.append(commandAngles)
    print str(commandAngles)
    centers = analyze_img()
    return [centers, motionAngles]


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
        ang = rot_angles[a][0] - (320 - centers[a][1])*RF
        # ang = ang.item()
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
##    print "im type: ", type(im)
##    return im


####################################
##''' This function will be deleted as it is only needed to
##call CenterOfMass. show_CM is just for visualization '''
##def show_CM(_name, CameraIndex):
##    img = cv2.imread(_name)
##    CM = CenterOfMass(img, CameraIndex)
##    # print CM
##    # cv2.circle(img, (CM[1], CM[0]), 2, (0, 255, 0), 3)
##    # cv2.imshow('detected ball', img)
##    # cv2.imshow(_name, img)
##    # cv2.waitKey(0)
##    # cv2.destroyAllWindows()
##    return CM
##################################


# Funtion that will look for position of the ball and return it
def initial_scan():
    state = 0
    angleSearch = 60*math.pi/180
    X = 0
    camIndex = 0 # Start with upper camera
    motion.moveInit()
    
    while X == 0 and state!=3:
        [CC, AA] = scan_area(angleSearch, camIndex)
        print 'Centers', CC
        [ang, X, delta] = rotate_center_head(CC, AA)
        print 'Angle', ang
        if X != 0:
            if abs(ang) > angleSearch + 0.0175:
                # error in angle calc back up and start scan over
                motion.moveTo(-0.1, 0, 0)
                X = 0
                state = 0
                continue
            elif X == 1:
                # saw ball only in one frame, turn to that frame and start scan over
                motion.moveTo(-0.1, 0, ang + 10 * math.pi/180)
                X = 0
                state = 0
                continue
        elif X == 0:
            if camIndex == 0:
                camIndex = 1
                continue
            elif camIndex == 1:
                state = state+1
                motion.moveTo(0, 0, 120 * math.pi/180)
                camIndex = 0
                continue
        # if X == 0:
        #     # didn't see the ball.  Increase state, turn 120 degrees 
        #     state = state+1
        #     motion.moveTo(0, 0, 120 * math.pi/180)
        # elif abs(ang) > angleSearch + 0.0175:
        #     # error in angle calc back up and start scan over
        #     motion.moveTo(-0.1,0,0)
        #     X=0
        #     state=0
        # elif X==1:
        #     ## saw ball only in one frame, turn to that frame and start scan over
        #     motion.moveTo(-0.1,0,ang+10*math.pi/180)
        #     X=0
        #     state=0
            


    if X==0:
        tts.say("I cannot find the ball.")
        CM=[0,0]
    else:
        zero_head()
        motion.moveTo(0, 0, ang*7/6)
        pic(path + "ball_upfront.png",0)
        img=cv2.imread(path + "ball_upfront.png")
        CM=CenterOfMassUp(img)
        tts.say("I found the ball.")
        
    return CM, delta, camIndex


def walkUp(cm, delta):
    idx = 1
    lowerFlag = 0
    print "Entering uppercam loop"
    motion.moveTo(0.2, 0, 0)
    while cm[0] < 420 and cm[0] > 0:
        pp = "ball_upfront"
        ext = ".png"
        # motion.moveTo(0.2, 0, 0)
        im_num = path + pp+str(idx)+ext
        pic(im_num, 0)
        img = cv2.imread(im_num)
        cm = CenterOfMassUp(img)
        print cm
        if cm[0] == 0 and cm[1] == 0:
            # Scan the area with lower camera
            pic(path + 'lower.png', 1)
            img = cv2.imread(path + "lower.png")
            cm2 = CenterOfMassUp(img)
            lowerFlag = 1
            break
        else:
            alpha = (cm[1] - 320) * delta
##            alpha = alpha.item()
            # print "Angle to rotate", alpha
            # motion.angleInterpolationWithSpeed("HeadYaw", alpha, 0.1)
            # zero_head()
            motion.moveTo(0.2, 0, alpha*7/6)
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
        pic(path + 'lower.png', 1)
        img = cv2.imread(path + 'lower.png')
        cm2 = CenterOfMassUp(img)
        lostFlag = 0
    motion.moveTo(0.15, 0, 0)
    print "Exiting up loop"
    return lostFlag, cm2
    # motion.moveTo(0.15, 0, 0)


def walkDown(cm, delta):
    # Do the correction before it starts the loop
    # alpha = (cm[1] - 320) * delta #######
##    alpha = alpha.item()
    # motion.moveTo(0, 0, alpha*7/6)
    idx = 1
    pp = "ball_downfront"
    ext = ".png"
    print 'Entering lowercam loop'
    # motion.moveTo(0.2, 0, alpha*7/6)
    motion.moveTo(0.2, 0, 0)
    while cm[0] > 0 and cm[0] < 230:
        # motion.moveTo(0.2, 0, 0)
        im_num = path + pp+str(idx)+ext
        pic(im_num, 1)
        img = cv2.imread(im_num)
        cm = CenterOfMassUp(img)
        print im_num, cm
        alpha = (cm[1] - 320) * delta
##        alpha = alpha.item()
        motion.moveTo(0.2, 0, alpha*7/6)
        idx = idx + 1
    # Tilt the head so it can have a better look of the ball
    anglePitch = math.pi * 20.6 / 180
    motion.angleInterpolationWithSpeed("HeadPitch", anglePitch, 0.1)
    print 'Pitching the head'
    # motion.moveTo(0.2, 0, 0)
    # The threshold of 300 is equal to a distance of 15cm from the ball
    # The robot will do a small walk of 7cm and exit the loop
    print 'Entering sub-precise with cm[0]', cm[0]
    while cm[0] >= 0  and cm[0] < 300:
        # motion.moveTo(0.2, 0, 0)
        im_num = path + pp+str(idx)+ext
        pic(im_num, 1)
        img = cv2.imread(im_num)
        cm = CenterOfMassDown(img)
        print im_num, cm
        if cm[0] < 350:
            alpha = (cm[1] - 320) * delta
    ##        alpha = alpha.item()
            motion.moveTo(0.07, 0, alpha*8/6)
        else: 
            break
        idx = idx + 1
    taskComplete = 1
    return taskComplete, cm


def getReady(cm, delta):
    # The ball should be at about 10cm roughly from the ball
    # Do the correction before it starts the loop
    idx = 1
    pp = "ball_precise"
    ext = ".png"
    im_num = path + pp+str(idx-1)+ext
    pic(im_num, 1)
    img = cv2.imread(im_num)
    cm = CenterOfMassDown(img)
    alpha = (cm[1] - 320) * delta
    # alpha = alpha.item()
    motion.moveTo(0, 0, alpha*7/6)
    print 'Precising the position'
    print 'This is my cm[0]', cm[0]
    while cm[0] < 370:
        print 'Entering the loop'
        # motion.moveTo(0.2, 0, 0)
        im_num = path + pp+str(idx)+ext
        pic(im_num, 1)
        img = cv2.imread(im_num)
        cm = CenterOfMassDown(img)
        print im_num, cm
        if cm[0] < 405:
            alpha = (cm[1] - 320) * delta
    ##        alpha = alpha.item()
            motion.moveTo(0.05, 0, alpha)
        else:
            break
        idx = idx + 1
    return 1
    # This should exit at a good distance to kick the ball


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

    #Wake up the robot
    motion.wakeUp()
    taskCompleteFlag = 0
    while taskCompleteFlag == 0:
        ballPosition, delta,camIndex =  initial_scan()
        if ballPosition==[0,0]:
            tts.say('I need to move to find the ball')
            motion.moveTo(0.5,0,0)
        else:
            if camIndex == 0:
                lost, CoM = walkUp(ballPosition, delta)
                if lost == 0:
                    # Switch cameras
                    time.sleep(0.2) 
                    video.stopCamera(0)
                    video.startCamera(1)
                    video.setActiveCamera(1)
                    # Walk to the ball using lower camera
                    taskCompleteFlag, CoM1 = walkDown(CoM, delta)
                    taskCompleteFlag = getReady(CoM1, delta)
                else:
                    tts.say('I lost the ball, I need to rescan.')
                    motion.moveTo(-0.2, 0, 0)
            else:
                # Switch cameras
                time.sleep(0.2)
                video.stopCamera(0)
                video.startCamera(1)
                video.setActiveCamera(1)
                # Walk to the ball using lower camera
                taskCompleteFlag, CoM1 = walkDown(CoM, delta)
                taskCompleteFlag = getReady(CoM1, delta)
    kickBall()
    motion.rest()
    # print commandAngles


if __name__ == "__main__":

    main(robotIP)
