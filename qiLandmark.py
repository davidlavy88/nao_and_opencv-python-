import time
import qi

robotIP = "192.168.1.122"
ses = qi.Session()
ses.connect(robotIP)
per = qi.PeriodicTask()
landmark = ses.service('ALLandMarkDetection')
memory = ses.service('ALMemory')
motion = ses.service('ALMotion')
posture = ses.service('ALRobotPosture')

period = 500
landmark.subscribe('Test_mark', period, 0.0)

memValue = 'LandmarkDetected'

# Wake Up the robot
motion.wakeUp()
motion.moveInit()

for i in range(0, 4):
    time.sleep(0.1)
    val = memory.getData(memValue, 0)
    print ""
    print "\*****"
    print ""
    # Check whether we got a valid output: a list with two fields.
    if(val and isinstance(val, list) and len(val) >= 2):
        # We detected landmarks !
        # For each mark, we can read its shape info and ID.
        # First Field = TimeStamp.
        timeStamp = val[0]
        # Second Field = array of Mark_Info's.
        markInfoArray = val[1]

        try:
            # Browse the markInfoArray to get info on each detected mark.
            for markInfo in markInfoArray:
                # First Field = Shape info.
                markShapeInfo = markInfo[0]
                # Second Field = Extra info (i.e., mark ID).
                markExtraInfo = markInfo[1]
                # Print Mark information.
                print "mark  ID: %d" % (markExtraInfo[0])
                print "  alpha %.3f - beta %.3f" % (markShapeInfo[1], markShapeInfo[2])
                print "  width %.3f - height %.3f" % (markShapeInfo[3], markShapeInfo[4])
        except Exception, e:
            print "Landmarks detected, but it seems getData is invalid. ALValue ="
            print val
            print "Error msg %s" % (str(e))
    else:
        print "Error with getData. ALValue = %s" % (str(val))

    # Unsubscribe from the module.
landmark.unsubscribe("Test_mark")
print "Test terminated successfully."
