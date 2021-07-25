from collections import deque
import cv2
import numpy as np
import imutils
import argparse
import pigpio
import math
from adafruit_servokit import ServoKit

#import RPi.GPIO as GPIO
import time
from time import sleep

import tkinter as tk

window = tk.Tk()

kit = ServoKit(channels=16)

#os.system("sudo pigpiod")


ap = argparse.ArgumentParser()
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
args=vars(ap.parse_args())
pts = deque(maxlen=args["buffer"])

cam = cv2.VideoCapture(-1)

#GPIO.setmode(GPIO.BOARD)
pi = pigpio.pi()

#GPIO.setup(11,GPIO.OUT)
#GPIO.setup(13,GPIO.OUT)
#servo1 = GPIO.PWM(11,50)
#servo2 = GPIO.PWM(13,50)

pi.set_mode(17, pigpio.OUTPUT)
pi.set_mode(27, pigpio.OUTPUT)

pi.set_servo_pulsewidth(17, 1500)
pi.set_servo_pulsewidth(27, 1500)
sleep(2)
pi.set_servo_pulsewidth(17, 0)
pi.set_servo_pulsewidth(27, 0)

xpidangle = 1500
ypidangle = 1500

xtarget = 300
ytarget = 300#230

Kp = 0.5 #best0.5
Ki = 0.01 #best0.01
Kd = 8#best8
errx = 0
erix = 0
errpx = 0
erry = 0
eriy = 0
errpy = 0

#xmin = 800
#ymin = 750
#xmax = 2200
#ymax = 2250

xmin = 1300
ymin = 1300
xmax = 1700
ymax = 1700

xmodel = 0
ymodel = 0

i = 0
delay = 10
delayedx = 0
delayedy = 0
xrdelay = 0
yrdelay = 0
delaylistx = []
delaylisty = []

starttime = time.time()
prevtime = 0
longesttime = 0

loop = True

while loop:
    
    #yellow
    #l_orange = np.array([30,30,80])
    #u_orange = np.array([35,225,205])
      
    #?green
    #l_orange = np.array([0,0,100])
    #u_orange = np.array([255,100,255])
    
    #dark blue
    #l_orange = np.array([200,0,50])
    #u_orange = np.array([255,255,75])
    
    #red
    l_orange = np.array([0,50,0])
    u_orange = np.array([10,255,255])
    
    b, img = cam.read()
    

    
    
    if b:
        rotated = cv2.rotate(img, cv2.ROTATE_180)
        final = cv2.convertScaleAbs(rotated, alpha = 3, beta  = 0)
        hsv = cv2.cvtColor(rotated, cv2.COLOR_BGR2HSV)
        masked = cv2.inRange(hsv, l_orange, u_orange)
        scary = cv2.bitwise_and(rotated, rotated, mask = masked)
        
        cnts = cv2.findContours(masked.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
        cnts = imutils.grab_contours(cnts)
        center = None
        
        
        if len(cnts) > 0:
        
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if M["m00"] > 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        
            if radius > 10:
           
                cv2.circle(rotated, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(rotated, center, 5, (0, 0, 255), -1)
                #modelBall.move()
                
                xr = round(x,1)
                yr = round(y,1)
                
                #delaylistx.append(modelBall.x)
                #delaylisty.append(yr)
                
                #if i > delay:
                    #delayedx = delaylistx.pop(0)
                    #delayedy = delaylisty.pop(0)
                #i += 1
                
                #xdelayedx = xr - delayedx
                #ydelayedy = yr - delayedy
                
                cv2.putText(rotated, ("position x %s " % xr+"position y %s" % yr), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 2)
                
                xrsum = xr#modelBall.x# + xdelayedx
                yrsum = yr#modelBall.y# + ydelayedy
                
                errx = -xtarget + (xrsum)
                erix = erix + errx
                if (erix > 30):
                    erix = 30
                errdx = errx - errpx
                errpx = errx
                
                erry = ytarget - (yrsum)
                eriy = eriy + erry
                if (eriy > 100):
                    eriy = 100
                errdy = erry - errpy
                errpy = erry
                
                xpidangle = ((Kp * errx + Ki * erix + Kd * (errdx)) * 1.5 + 1500)
                #xpidangle = 1500
                #ypidangle = 1500
                ypidangle = ((Kp * erry + Ki * eriy + Kd * (errdy)) * 1.5 + 1500)
                
                
                if xpidangle < xmin:
                    xpidangle = xmin
                if ypidangle < ymin:
                    ypidangle = ymin
                if xpidangle > xmax:
                    xpidangle = xmax
                if ypidangle > ymax:
                    ypidangle = ymax

                
                #modelBall.draw(modelBall.x,modelBall.y,rotated)
                
                    
                pi.set_servo_pulsewidth(17, xpidangle)
                pi.set_servo_pulsewidth(27, ypidangle)
                
                #print (time)
                
        else:
            pi.set_servo_pulsewidth(17, 1500)
            pi.set_servo_pulsewidth(27, 1500)
                
                #pi.stop()
                
        pts.appendleft(center)
        
        
        
        cv2.imshow("Window 1",rotated)
        #cv2.imshow("Window 2",final)
        #cv2.imshow("Window 3",masked)
        #cv2.imshow("Window 4",scary)

        print(xpidangle)
        print('probka: ')
        print(time.time() - prevtime)
                
        if 0.08 >(time.time() - prevtime)>longesttime:
            longesttime = (time.time() - prevtime)
                
        prevtime = time.time()
                
                    
        print('l: ')
        print(longesttime)
                
        cv2.waitKey(1)
        
        
        
        
    else:
        print("The camera is not working!")
        break
    
    key = cv2.waitKey(1)&0xFF
    if key==ord('q'):
        break
    
cv2.destroyAllWindows()
cam.release()

