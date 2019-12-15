#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Dec  6 17:38:10 2019

@author: mioooo
"""

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Dec  4 12:30:06 2019

@author: mioooo
"""

import cv2
import numpy as np


video = cv2.VideoCapture("test3.mp4")



lower_hue_test1 = np.array([19,0,205])
upper_hue_test1 = np.array([255,14,255])

def processImage(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv ,lower_hue_test1,upper_hue_test1)
    canny1= cv2.Canny(mask,50,150)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    canny = cv2.Canny(blur, 50, 150)
    return canny1

def region_of_interest(image):
    height = image.shape[0]
    polygons = np.array([
    [(0,height), (3000,height), (750,250)]
    ])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask) 
    return masked_image

def display_lines(image, lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            cv2.line(line_image,(x1, y1), (x2, y2), (255,0,0), 10)
    return line_image

def average_slope_intercept(image, lines):
    left_fit = []
    right_fit = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))
        left_fit_average = np.average(left_fit, axis=0)
        right_fit_average = np.average(right_fit, axis=0)
        #print(left_fit_average, 'left')
        #print(right_fit_average, 'right')
        left_line = make_coordinates(image, left_fit_average)
        right_line = make_coordinates(image, right_fit_average)
        return np.array([left_line, right_line])

def make_coordinates(image, line_parameters):
    try:
        slope, intercept = line_parameters
    except TypeError:
        slope, intercept = 1,0
    y1 = image.shape[0]
    y2 = int(y1*3/5)
    x1 = int(y1 - intercept)/slope
    x2 = int(y2 - intercept)/slope
    #print(x1)
    #print(x2)      
        
    return np.array([x1, y1, x2, y2])

while True:

    _,frame = video.read()
    grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    processed_image = processImage(frame)
    cropped_image = region_of_interest(processed_image)
    lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 60, np.array([]), minLineLength=40, maxLineGap=5)
    averaged_lines = average_slope_intercept(grayFrame, lines)
    line_image = display_lines(cropped_image,lines) 
    combo_image = cv2.addWeighted(grayFrame, .6, line_image, 1, 1)
    cv2.imshow('result1', cropped_image)
    cv2.imshow('result2', combo_image)
    cv2.imshow('result3', line_image)
    if lines is None:
        print (None)
    else:
        print(lines)
        x3 =np.max(lines)
        x4 = np.min(lines)
        print(type(x3))
        set_poiont = 555
        sum_x4_x5 =(np.int32(x3) + np.int32(x4))/2
        error = set_poiont-sum_x4_x5
        print(error)
        

    if cv2.waitKey(30) & 0xFF == ord('q'):
        break


cv2.destroyAllWindows()
