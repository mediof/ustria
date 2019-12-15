#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Dec  3 17:39:12 2019

@author: mioooo
"""
import cv2
from matplotlib import pyplot as plt
import numpy as np

def make_coordinates(image,line_parameter):
    slope,intercept = (line_parameter)
    y1=image.shape[0]
    y2=int(y1*(3/5))
    x1=int((y1-intercept)/slope)
    x2=int((y2-intercept)/slope)
    return np.array([x1,y1,x2,y2])

def average_of_intercept(image,lines):
    left_fit = []
    right_fit = []
    for line in lines:
        x1,y1,x2,y2=line.reshape(4)
        parameters = np.polyfit((x1,x2),(y1,y2),1)
        slope = parameters[0]
        intercept = parameters[1]
        if slope<0:
            left_fit.append((slope,intercept))
        else:
            right_fit.append((slope,intercept))
    #print(left_fit)
    #print(right_fit)  
    left_fit_average = np.average(left_fit,axis = 0 )
    right_fit_average = np.average(right_fit,axis = 0 )

    left_line = make_coordinates(image,left_fit_average)
    right_line = make_coordinates(image , right_fit_average)
    return np.array([left_line,right_line])
        
def canny(image):
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    blur= cv2.GaussianBlur(gray,(5,5),0)
    canny = cv2.Canny(blur,50,150)
    return canny


def display_lines(image,lines):
    lines_image=np.zeros_like(image)
    if lines is not None:
        for x1,y1,x2,y2 in lines:
            cv2.line(lines_image,(x1,y1),(x2,y2),(255,0,0),10)
    return lines_image
    
def region_of_interest(image):
    height = image.shape[0]
    polygons = np.array([
    [(200,height), (1200,height), (750,300)]
    ])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask) 
    return masked_image

cap = cv2.VideoCapture("test2.mp4")
while(cap.isOpened()):
    _,frame =cap.read()
    canny_image = canny(frame)
    cropped_image = region_of_interest(canny_image)
    lines = cv2.HoughLinesP(cropped_image,2,np.pi/180,100,np.array([]),minLineLength=40 ,maxLineGap=5)
    averaged_lines = average_of_intercept(frame,lines)
    lines_image = display_lines(frame,averaged_lines)
    combo_image = cv2.addWeighted(frame,0.8,lines_image,1,1)

    cv2.waitKey(1) 
    

    

    
    