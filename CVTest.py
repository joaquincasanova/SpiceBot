import cv2
import numpy as np
import os
import time
def thresh_adjust_inv(mat):
    retval,thresh = cv2.threshold(mat,127,255,cv2.THRESH_BINARY_INV)
    
    cv2.namedWindow('thresh',cv2.WINDOW_NORMAL)
    cv2.createTrackbar('T','thresh',0,255,nothing)
    while(1):
        cv2.imshow('thresh',thresh)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

            # get current positions of four trackbars
        T = cv2.getTrackbarPos('T','thresh')
        retval,thresh = cv2.threshold(mat,T,255,cv2.THRESH_BINARY_INV)    

    retval,thresh = cv2.threshold(mat,T,1,cv2.THRESH_BINARY_INV)    
    cv2.destroyAllWindows()

    return thresh, T
    
def canny_adjust(mat):
    edges = cv2.Canny(mat,50,150)
    cv2.namedWindow('edges',cv2.WINDOW_NORMAL)
    cv2.createTrackbar('MinVal','edges',0,1000,nothing)
    cv2.createTrackbar('MaxVal','edges',0,1000,nothing)
    while(1):
        cv2.imshow('edges',edges)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

            # get current positions of four trackbars
        minval = cv2.getTrackbarPos('MinVal','edges')
        maxval = cv2.getTrackbarPos('MaxVal','edges')
            
        edges = cv2.Canny(mat,minval,maxval)

    cv2.destroyAllWindows()

    return edges, minval, maxval
    
def hough_adjust(mat,img0):
    img=np.copy(img0)
    lines = cv2.HoughLinesP(mat,1,np.pi/180,200,100,10)
    print lines
    if lines!=None:
        for x1,y1,x2,y2 in lines[0]:
            cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
    
    cv2.namedWindow('lines',cv2.WINDOW_NORMAL)
    cv2.createTrackbar('Threshhold','lines',1,1000,nothing)
    cv2.createTrackbar('MinLL','lines',1,1000,nothing)
    cv2.createTrackbar('MinLG','lines',1,1000,nothing)
    cv2.imshow('lines',img)
    while(1):
        cv2.imshow('lines',img)
        img = np.copy(img0)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
            
            # get current positions of four trackbars
        
        Threshhold = (cv2.getTrackbarPos('Threshhold','lines'))
        MinLL = (cv2.getTrackbarPos('MinLL','lines'))
        MinLG = (cv2.getTrackbarPos('MinLL','lines'))
        
        lines = cv2.HoughLinesP(mat,1,np.pi/180,Threshhold,MinLL,MinLG)
        print lines
        if lines!=None:
            for x1,y1,x2,y2 in lines[0]:
                cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
                
    cv2.destroyAllWindows()

    return lines, Threshhold, MinLL, MinLG

def bilat_adjust(mat):

    bilat = cv2.bilateralFilter(mat, -1, 7, 7)

    cv2.namedWindow('bilat',cv2.WINDOW_NORMAL)
    cv2.createTrackbar('sigC','bilat',0,100,nothing)
    cv2.createTrackbar('sigD','bilat',0,100,nothing)
    while(1):
        cv2.imshow('bilat',bilat)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
  
        sigC = cv2.getTrackbarPos('sigC','bilat')
        sigD = cv2.getTrackbarPos('sigD','bilat')
        bilat = cv2.bilateralFilter(mat, -1, sigC, sigD)

    cv2.destroyAllWindows()

    return bilat, sigC, sigD

def nothing(x):
    pass

def normalize(x):
    y=(x-np.min(x))/(np.max(x)-np.min(x))*255
    return y

img = cv2.imread('webcam.jpg')

hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
h,s,v = cv2.split(hsv)
heq=cv2.equalizeHist(h)
hb = heq
#hb, sigC, sigD = bilat_adjust(heq)
#vt, T = thresh_adjust_inv(hb)
#hc = cv2.Canny(hb,500,160)
hc, minval, maxval = canny_adjust(hb)
hl, thresh, minll, minlg = hough_adjust(hc,img)
