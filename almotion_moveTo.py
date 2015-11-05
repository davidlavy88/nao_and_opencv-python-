# -*- encoding: UTF-8 -*-

'''Move To: Small example to make Nao Move To an Objective'''

import argparse
import math
import almath as m # python's wrapping of almath
from naoqi import ALProxy


def main(robotIP, PORT=9559):

    motionProxy  = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

    # Wake up robot
    motionProxy.wakeUp()

    # Send robot to Stand Init
    postureProxy.goToPosture("StandInit", 0.5)

    #####################
    ## Enable arms control by move algorithm
    #####################
    motionProxy.setMoveArmsEnabled(True, True)
    #~ motionProxy.setMoveArmsEnabled(False, False)

    #####################
    ## FOOT CONTACT PROTECTION
    #####################
    #~ motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION",False]])
    motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

    #####################
    ## get robot position before move
    #####################
    initRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False))

    X = 0.0
    Y = 0.0
    Theta = math.pi/2.0
    motionProxy.post.moveTo(X, Y, Theta)
    motionProxy.setAngles("HeadYaw", 0.6, 0.6)
    # wait is useful because with post moveTo is not blocking function
    motionProxy.waitUntilMoveIsFinished()

    #####################
    ## get robot position after move
    #####################
    endRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False))

    #####################
    ## compute and print the robot motion
    #####################
    robotMove = m.pose2DInverse(initRobotPosition)*endRobotPosition
    # return an angle between ]-PI, PI]
    robotMove.theta = m.modulo2PI(robotMove.theta)
    print "Robot Move:", robotMove

    # Go to rest position
    motionProxy.rest()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.1.107",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)