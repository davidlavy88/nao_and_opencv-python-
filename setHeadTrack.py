import qi
import naoqi
import argparse 
import time
import random
import math

# Python Image Library
import Image

import numpy as np
import cv2

#robotIP = "192.168.1.107"
robotIP = "169.254.252.60"
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

def scan_area():
	# Start the client for acquisition
	videoClient = video.subscribe("python_client", resolution, colorSpace, 5)
	# Move neck to extreme right
	initAngle = -math.pi/2
	maxAngle = math.pi/2
	rotationAngle = math.pi/6
	motion.setAngles("HeadYaw", initAngle, 0.6)
	time.sleep(0.8)
	# Six seconds for a complete scan, taking 5 pictures per second
	testTime = 6
	t = 0
	dt = 1.2
	s = "camImage"
	ext = ".png"
	n = 1
	angle = initAngle
	#while t < testTime:
	while 
		naoImage = video.getImageRemote(videoClient)
		# Get the image size and pixel array.
	  	imageWidth = naoImage[0]
	  	imageHeight = naoImage[1]
	  	array = naoImage[6]
		im = Image.fromstring("RGB", (imageWidth, imageHeight), str(array))
		name = s+str(n)+ext
		im.save(name, "PNG")
		img = np.array(im) 
	  	# Convert RGB to BGR 
	  	img = img[:, :, ::-1].copy()
	  	angle = angle + rotationAngle
	  	motion.setAngles("HeadYaw", initAngle, 0.6)
	  	t = t + dt
	  	n = n + 1
	  	time.sleep(0.8)



def move_head():
	motion.setPositions('Head', 1, [0,0,0,0,0,0], 0.5, 56)

def move_body():
	motion.moveTo(0,0,1)

def move_both():
	per.start(True)
	qi.async(move_body)

def  main(robotIP, PORT=9559):

	motion.wakeUp()
	posture.goToPosture('StandInit',0.5)

	scan_area()

	# videoClient = video.subscribe("python_client", resolution, colorSpace, 5)
	# t0 = time.time()

	# # Get a camera image.
	# # image[6] contains the image data passed as an array of ASCII chars.
	# naoImage = video.getImageRemote(videoClient)

	# t1 = time.time()

	# # Time the image transfer.
	# print "acquisition delay ", t1 - t0

	# video.unsubscribe(videoClient)


	# # Now we work with the image returned and save it as a PNG  using ImageDraw
	# # package.

	# # Get the image size and pixel array.
	# imageWidth = naoImage[0]
	# imageHeight = naoImage[1]
	# numLayers = naoImage[2]
	# array = naoImage[6]

	# # Create a PIL Image from our pixel array.
	# im = Image.fromstring("RGB", (imageWidth, imageHeight), str(array))
	# #nparr = np.fromstring(array, np.uint8).reshape( imageHeight, imageWidth, numLayers)
	# #img_np = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_COLOR)

	# #cv2.imshow('np',img_np)

	# #print type(img_np)
	# # Save the image.
	# im.save("camImage2.png", "PNG")

	# im.show()


	# #motion.moveTo(0,0,-1)
	# #move_head()

	# per.setUsPeriod(500000)
	# per.setCallback(move_head)
	# per.start(True)

	# motion.moveTo(0,0,-1)
	# #motion.setPositions('Head', 1, [0,0,0,0,0,1.5], 0.5, 56)
	# time.sleep(3)

	# motion.moveTo(0,0,1)
	# #motion.setPositions('Head', 1, [0,0,0,0,0,1.5], 0.5, 56)
	# time.sleep(3)

	# #move_both()
	# #motion.moveTo(0,0,-1)
	# #motion.moveTo(0,0,1)

	# per.stop()

	#stop move on the next double support
    #motion.stopMove()

    #Go to rest position
   	motion.rest()


if __name__ == "__main__":   
    
    #169.254.252.60
    main(robotIP)