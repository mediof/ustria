#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Dec  1 13:31:53 2019

@author: mioooo
"""

import cv2
import numpy as np
def nothing(x):
    pass
cap = cv2.VideoCapture("test3.mp4")
cv2.namedWindow("Trackbars")
cv2.createTrackbar("L - H", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", 0 ,255, nothing)
cv2.createTrackbar("U - S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars",0, 255, nothing)
while True:
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    l_h = cv2.getTrackbarPos("L - H", "Trackbars")
    l_s = cv2.getTrackbarPos("L - S", "Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "Trackbars")
    u_h = cv2.getTrackbarPos("U - H", "Trackbars")
    u_s = cv2.getTrackbarPos("U - S", "Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "Trackbars")
    lower_blue = np.array([l_h, l_s, l_v])
    upper_blue = np.array([u_h, u_s, u_v])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    result = cv2.bitwise_and(frame, frame, mask=mask)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray , (5,5),0)
    edge = cv2.Canny(blur,50,150)
    lines = cv2.HoughLinesP(edge,1,np.pi/360 ,60,160 , 3)
    
    

    #for x1,y1,x2,y2 in lines[0]:
    #    cv2.line(frame,(x1,y1) , (x2,y2) , (0,255,0) , 2)
            
    cv2.imshow("line_detect_test " ,frame)
 
    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)
    cv2.imshow("result", result)
    key = cv2.waitKey(1)
    if key == 27:
        break
cap.release()
cv2.destroyAllWindows()