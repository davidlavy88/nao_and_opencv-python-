import qi
import time
import functools

#assume we have a connected session

robotIP = "192.168.1.107"
ses = qi.Session()
ses.connect(robotIP)

tts = ses.service("ALTextToSpeech")
sayHelloCallable = functools.partial(tts.say, "hello")

sayHelloTask = qi.PeriodicTask()
sayHelloTask.setCallback(sayHelloCallable)
sayHelloTask.setUsPeriod(1200000)
sayHelloTask.start(True)

time.sleep(6)
sayHelloTask.stop()