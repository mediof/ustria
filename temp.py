from __future__ import print_function
import pyzbar.pyzbar as pyzbar
import numpy as np
import cv2
import time
from djitellopy import Tello
import os
import sys
import socket
import threading




tello_address = ('192.168.10.1', 8889)
local_address = ('', 9000)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(local_address)

def send(message, delay):

  try:
    sock.sendto(message.encode(), tello_address)
    print("Sending message: " + message)
  except Exception as e:
    print("Error sending: " + str(e))

  time.sleep(delay)

def receive():

  while True:

    try:
      response, ip_address = sock.recvfrom(128)
      print("Received message: " + response.decode(encoding='utf-8'))
    except Exception as e:

      sock.close()
      print("Error receiving: " + str(e))
      break


tello = Tello()

S = 20
FPS = 120

distanceThreshold = 250


speed = 10

currentScreenshot = 1

def decode(im):
    # Find barcodes and QR codes
    decodedObjects = pyzbar.decode(im)
    # Print results
    for obj in decodedObjects:
        print('Type : ', obj.type)
        print('Data : ', obj.data, '\n')
        if obj.data == "4" :
            time.sleep(6)
            send("command",3)
        
            send("cw " + str(+90), 10)
            
        if obj.data == "4" :
            time.sleep(6)
            send("command",3)
            time.sleep(2)
            send("cw " + str(+90), 10)
            time.sleep(3)
            send("forward " + str(200),5)
            
        
        
    return decodedObjects

mask1 = np.zeros((480,640), np.uint8)
mask1[0:480,280:360] = 255






font = cv2.FONT_HERSHEY_SIMPLEX
counterLeft = 0
counterRight = 0
counterUp = 0
counterDown = 0

def takePicture ():
    os.system("screencapture screenshot" + str(currentScreenshot) + ".png")
    currentScreenShot = currentScreenshot + 1

if not tello.connect():
    print("Tello not connected")
    sys.exit()

if not tello.set_speed(speed):
    print("Not set speed to lowest possible")
    sys.exit()

# In case streaming is on. This happens when we quit this program without the escape key.
if not tello.streamoff():
    print("Could not stop video stream")
    sys.exit()

if not tello.streamon():
    print("Could not start video stream")
    sys.exit()

print ("Current battery is " + tello.get_battery())

frame_read = tello.get_frame_read()
stop = False

lower_hue = np.array([45,50,12])
upper_hue = np.array([181,255,255])

up_black = np.array([255,255,255])
low_black = np.array([200,200,200])       
#send("command",3)
#send("takeoff", 5)
kernel = np.ones((5,5),np.uint8)
        
while not stop:

    frame = frame_read.frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    gaus = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115, 1)

    mask = cv2.inRange(hsv ,lower_hue,upper_hue)
    #mask=cv2.morphologyEx(mask , cv2.MORPH_CLOSE ,kernel)
    edge = cv2.Canny(mask,50,150)
    lines = cv2.HoughLinesP(edge,1,np.pi/180 , 20,80 , 2)
    lines = masked_image = cv2.bitwise_and(lines, mask1) 
    
    if lines is not None:
        send("command",0)
        send("forward " + str(300),0)
    
      
    decodedObjects = decode(gaus)

    for decodedObject in decodedObjects:
        points = decodedObject.polygon
    cv2.imshow("Video", mask)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

    # time.sleep(1/FPS)

tello.land()
tello.streamoff()
cv2.destroyAllWindows()
sys.exit()
