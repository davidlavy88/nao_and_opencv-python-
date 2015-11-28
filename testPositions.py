import almath
import argparse
import qi

robotIP = "192.168.1.107"

ses = qi.Session()
ses.connect(robotIP)
per = qi.PeriodicTask()
scan = qi.PeriodicTask()
motion = ses.service('ALMotion')
posture = ses.service('ALRobotPosture')
tracker = ses.service('ALTracker')
video = ses.service('ALVideoDevice')

def main(robotIP, PORT=9559):
    # motion.wakeUp()
    # motion.angleInterpolationWithSpeed( "Head", [ang, 0.035 ], 0.1 );
    useSensorValues = False
    result = motion.getRobotPosition(useSensorValues)
    print "Robot Position ", result
    motion.rest()



if __name__ == "__main__":
    main(robotIP)
