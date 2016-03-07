#Get histograms from multiple images of the ball and find an average histogram
#for h and s space


import sys
import time
import cv2
import numpy
import matplotlib.pyplot as plt
from collections import deque

# Python Image Library
from PIL import Image

from naoqi import ALProxy


def shift(key, array):
    a = deque(array) # turn list into deque
    a.rotate(key)    # rotate deque by key
    return list(a)   # turn deque back into a list

def hsvHist(image,binsh,binss):
    # change to hsv colorspace
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img2 = image[:, :, ::-1].copy()
    
    binsv = 25

    lowera = numpy.array([160,95,0])
    uppera = numpy.array([180,250,255])
    lowerb = numpy.array([0,95,0])
    upperb = numpy.array([10,250,255])

    mask1 = cv2.inRange(hsv, lowera, uppera)
    mask2 = cv2.inRange(hsv, lowerb, upperb)

    mask = cv2.add(mask1,mask2)

    #maska = numpy.zeros(hsv.shape[:2], numpy.uint8)
    #maska[230:340, 230:340] = 255
    #mask = cv2.bitwise_and(mask,mask,mask = maska)

    kernel = numpy.ones((5,5),numpy.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    res = cv2.bitwise_and(img2,img2, mask= mask)

    histh_mask = cv2.calcHist([hsv],[0],mask,[binsh],[0,179])
    hists_mask = cv2.calcHist([hsv],[1],mask,[binss],[0,256])
    histv_mask = cv2.calcHist([hsv],[2],mask,[binsv],[0,256])
    hist_hs=[[histh_mask],[hists_mask]]

    plt.subplot(221), plt.imshow(img2, 'gray')
    plt.title('Original Image')
    plt.subplot(222), plt.imshow(res, 'gray')
    plt.title('Masked Image')
    plt.subplot(223), plt.plot(histh_mask)
    plt.title('Histogram of H, 18 bins')
    plt.xlim([0,binsh-1])
    plt.subplot(224), plt.plot(hists_mask)
    plt.title('Histogram of S, 25 bins')
    plt.xlim([0,binss-1])

    plt.show()
    return hist_hs

if __name__ == '__main__':
    names=["ball0.png","15cm.jpg","tencm.jpg","test6.png"]
    img = cv2.imread("ball0.png")
    binsh = 18
    binss = 25
    hs = hsvHist(img,binsh,binss)
    h_hist0=hs[0][0]
    h_hist0_normalized=h_hist0/sum(h_hist0)
    s_hist0=hs[1][0]
    s_hist0_normalized=s_hist0/sum(s_hist0)
    img = cv2.imread("test6.png")
    hs = hsvHist(img,binsh,binss)
    h_hist1=hs[0][0]
    h_hist1_normalized=h_hist1/sum(h_hist1)
    s_hist1=hs[1][0]
    s_hist1_normalized=s_hist1/sum(s_hist1)
    img = cv2.imread("15cm.jpg")
    hs = hsvHist(img,binsh,binss)
    h_hist2=hs[0][0]
    h_hist2_normalized=h_hist2/sum(h_hist2)
    s_hist2=hs[1][0]
    s_hist2_normalized=s_hist2/sum(s_hist2)
    img = cv2.imread("tencm.jpg")
    hs = hsvHist(img,binsh,binss)
    h_hist3=hs[0][0]
    h_hist3_normalized=h_hist3/sum(h_hist3)
    s_hist3=hs[1][0]
    s_hist3_normalized=s_hist3/sum(s_hist3)
    h_hist_avg=(h_hist0_normalized+h_hist1_normalized+h_hist2_normalized+h_hist3_normalized)/4
    s_hist_avg=(s_hist0_normalized+s_hist1_normalized+s_hist2_normalized+s_hist3_normalized)/4

    for j in range(len(s_hist_avg)-2):
        s_hist_avg[j+1][0]=s_hist_avg[j][0]*2/5+s_hist_avg[j+1][0]*1/5+s_hist_avg[j+2][0]*2/5
    
    
    plt.subplot(121), plt.plot(h_hist_avg)
    plt.title('Average H Histogram')
    plt.xlim([0,binsh-1])
    plt.subplot(122), plt.plot(s_hist_avg)
    plt.title('Average S Histogram Smoothed')
    plt.xlim([0,binss-1])
    plt.show()







    
