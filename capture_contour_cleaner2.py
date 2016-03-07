#capture image, get center of mass

import sys
import time
import math
import cv2
import numpy as np

# Python Image Library
from PIL import Image

from naoqi import ALProxy
import vision_definitions
import time
# from matplotlib import pyplot as plt

def getNaoImage(IP, PORT):
    
    camProxy = ALProxy("ALVideoDevice", IP, PORT)
    
    resolution = 2 # 640*480px http://doc.aldebaran.com/2-1/family/robots/video_robot.html#cameraresolution-mt9m114
    colorSpace = 11 # RGB colorspace http://doc.aldebaran.com/2-1/family/robots/video_robot.html#cameracolorspace-mt9m114
    fps = 5 # can be 0-30 fps

    videoClient = camProxy.subscribe("python_client", resolution, colorSpace, fps)
    t0 = time.time()
    naoImage = camProxy.getImageRemote(videoClient)
    t1 = time.time()
    
    camProxy.unsubscribe(videoClient)

    # Get the image size and pixel array.
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]

    # Create a PIL Image from our pixel array.
    im = Image.fromstring("RGB", (imageWidth, imageHeight), array)
    #grab image from PIL and convert to opencv image
    img = np.array(im)
    img = img[:, :, ::-1].copy()

    #im.save(name,"PNG")
    
    
    print "acquisition delay ", t1 - t0
    return img

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
    if len(cont)>=1:
        maxim=0
        for i in range(len(cont)):
            if len(cont[i])>len(cont[maxim]):
                maxim=i
        contour=cont[maxim]
    
        maxi=np.amax(contour,0)
        mini=np.amin(contour,0)
        center=(maxi+mini)/2
        i=center[0][1]
        j=center[0][0]
##        d=abs(maxi[0][0]-center[0][0])
##        err=[]
##        for i in range(len(contour)):
##            t=[contour[i,0,0]-center[0,0],contour[i,0,1]-center[0,1]]
##            t=math.sqrt(math.pow(t[0],2)+math.pow(t[1],2))
##            e=abs(d-t)/d
##            err=err+[e]
##        ERR=np.var(err)
##        print ERR
##        print np.mean(err)
##        if ERR>.007:
##            i=0
##            j=0
##        else:
##            i=center[0][1]
##            j=center[0][0]
    else:
        i=0
        j=0
        contour=cont

    CM=[i,j]
    # remove output of contour for use in code
    return CM, contour


if __name__ == '__main__':
  #IP = "169.254.252.60"  # Replace here with your NaoQi's IP address.
  #PORT = 9559
  #
  # Read IP address from first argument if any.
  #if len(sys.argv) > 1:
  image = sys.argv[1]
  #im=getNaoImage(IP, PORT, "test0.png")
  img = cv2.imread(image)
  CM,cont=CenterOfMassDown(img)
##  print cont[0][0][0]

  cv2.drawContours(img, cont, -1, (0,190,255), 3)
  cv2.circle(img,(CM[1],CM[0]),2,(0,255,0),3)
  cv2.imshow('detected ball',img)
  cv2.waitKey(0)
  cv2.destroyAllWindows()
  #img.save(name,"PNG")
