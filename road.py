#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 27 17:09:03 2019

@author: mehdi
"""

import cv2
import numpy as np

# get the video Data
video = cv2.VideoCapture(0)



# starting the detection
while True:
    ret , frame = video.read()
    gray = cv2.cvtColor(frame , cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray , (5,5),0)
    edge = cv2.Canny(blur,50,150)
    lines = cv2.HoughLinesP(edge,1,np.pi/180 , 60,80 , 1)
    
    

    for x1,y1,x2,y2 in lines[0]:
        cv2.line(frame,(x1,y1) , (x2,y2) , (0,255,0) , 2)
            
    cv2.imshow("line_detect_test " ,frame)
    print (lines)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

    # time.sleep(1/FPS)

cv2.destroyAllWindows()
