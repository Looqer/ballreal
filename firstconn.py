from collections import deque
import cv2
import numpy as np
import imutils
import argparse
import pigpio
import math

#import RPi.GPIO as GPIO
import time
from time import sleep


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
ytarget = 250

Kp = 1
Kd = 0.75
errx = 0
errpx = 0
erry = 0
errpy = 0

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

class BallModel:
    
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.mass = 1
        self.grav = 9.81
        self.speed = 0
        self.xv = 0
        self.yv = 0
        
        
    def draw(self, x, y, screen):
        
        cv2.circle(screen, (int(self.x), int(self.y)), 10, (0,0,255))
        
    def move(self):
        
        #friction
        self.speed -= (self.speed)/50

        #stop if movement is slow
        if self.speed <= .01:
            self.speed = 0
            
        #Get servo pose
        slidexangle = -xpidangle*3.14/750+2*3.14
        slideyangle = -ypidangle*3.14/750+2*3.14
        
        print ("xangle: " ,slidexangle)
            
        # Acceleration calculation
        self.xF = self.mass * self.grav * math.sin(slidexangle)
        self.yF = self.mass * self.grav * math.sin(slideyangle)
        
        print ("xforce: " ,self.xF)

        #Acceleration calculation
        self.xa = self.xF/self.mass
        self.ya = self.yF/self.mass

        #Velocity calculation
        self.xv += self.xa
        self.yv += self.ya
        
        #change position with velocity
        self.x += self.xv
        self.y = 250#self.y + self.yv
        
        if self.x < 150:
            self.x = 150
        if self.y < 100:
            self.y = 100
        if self.x > 450:
            self.x = 450
        if self.y > 400:
            self.y = 400
            
        
    
modelBall = BallModel(200,200)
loop = True

while loop:
    
    
    
    
    
    l_orange = np.array([45,100,50])
    u_orange = np.array([75,255,255])
    
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
                modelBall.move()
                
                xr = round(x,1)
                yr = round(y,1)
                
                delaylistx.append(modelBall.x)
                delaylisty.append(yr)
                
                if i > delay:
                    delayedx = delaylistx.pop(0)
                    delayedy = delaylisty.pop(0)
                i += 1
                
                xdelayedx = xr - delayedx
                ydelayedy = yr - delayedy
                
                cv2.putText(rotated, ("position x %s " % xr+"position y %s" % yr), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 2)
                
                xrsum = modelBall.x + xdelayedx
                yrsum = modelBall.y + ydelayedy
                
                errx = xtarget - (xrsum)
                errdx = errx - errpx
                errpx = errx
                
                erry = ytarget - (yrsum)
                errdy = erry - errpy
                errpy = erry
                
                xpidangle = ((Kp * errx + Kd * (errdx)) * 5 + 1500)
                ypidangle = ((Kp * erry + Kd * (errdy)) * 5 + 1750)
                
                
                if xpidangle < 800:
                    xpidangle = 800
                if ypidangle < 750:
                    ypidangle = 750
                if xpidangle > 2200:
                    xpidangle = 2200
                if ypidangle > 2250:
                    ypidangle = 2250
                    
                
                
                modelBall.draw(modelBall.x,modelBall.y,rotated)
                
                pi.set_servo_pulsewidth(17, xpidangle)
                pi.set_servo_pulsewidth(27, 1500) #ypidangle)
                
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

        
        cv2.waitKey(1)
        
    else:
        print("The camera is not working!")
        break
    
    key = cv2.waitKey(1)&0xFF
    if key==ord('q'):
        break
    
cv2.destroyAllWindows()
cam.release()

