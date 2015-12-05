#capture image, get center of mass
 
import sys
import time
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
 
def CenterOfMass(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img2 = image[:, :, ::-1].copy()
 
    #100 lower range of saturation for daveed's place
    #195 lower range of saturation for travis' place
    lowera = np.array([160,165,0])
    uppera = np.array([180,250,255])
    lowerb = np.array([0,165,0])
    upperb = np.array([5,250,255])
 
    mask1 = cv2.inRange(hsv, lowera, uppera)
    mask2 = cv2.inRange(hsv, lowerb, upperb)
 
    mask = cv2.add(mask1,mask2)
 
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    cv2.imshow("mask",mask)
 
    mj=sum(mask)
    mi=sum(np.transpose(mask))
    A=mask.shape
    ni=np.array(range(A[0]))
    nj=np.array(range(A[1]))
    M=sum(sum(mask))
    if sum(mj)==0 or sum(mi)==0:
        print "no ball"
        xcm=0
        ycm=0
    else:
        xcm=np.dot(mj,nj)/sum(mj)
        ycm=np.dot(mi,ni)/sum(mi)
     
    CM=[ycm,xcm]
     
    return CM
 
if __name__ == '__main__':
  #IP = "169.254.252.60"  # Replace here with your NaoQi's IP address.
  #PORT = 9559
  #
  # Read IP address from first argument if any.
  if len(sys.argv) > 1:
     picname = sys.argv[1]
     picname = str(picname)
  #im=getNaoImage(IP, PORT, "test0.png")
  # img = cv2.imread("ball_upfront2.png")
  img = cv2.imread(picname)
  CM=CenterOfMass(img)
  print CM
 
  cv2.circle(img,(CM[1],CM[0]),2,(0,255,0),3)
  cv2.circle(img,(320,240),2,(255,0,0),3)
  cv2.imshow('detected ball',img)
  cv2.waitKey(0)
  cv2.destroyAllWindows()