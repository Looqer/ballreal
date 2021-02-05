from collections import deque
import cv2
import numpy as np
import imutils
import argparse
import pigpio

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


#servo1 = GPIO.PWM(11,50)
#servo2 = GPIO.PWM(13,50)
#servo1.start(6)
#servo2.start(6)



xservopos = 1500
yservopos = 1500

Kp = 1
Kd = 0.75
errx = 0
errpx = 0
erry = 0
errpy = 0

while True:
    
    

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
                xr = round(x,1)
                yr = round(y,1)
                cv2.putText(rotated, ("position x %s " % xr+"position y %s" % yr), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 2)
                
                
                errx = (xr) - 300
                errdx = errx - errpx
                errpx = errx
                
                erry = (yr) - 250
                errdy = erry - errpy
                errpy = erry
                
                xservopos = ((Kp * errx + Kd * (errpx)) * 5 + 1500)
                yservopos = ((Kp * erry + Kd * (errpy)) * 5 + 1750)
                
                
                if xservopos < 500:
                    xservopos = 500
                if yservopos < 500:
                    yservopos = 500
                if xservopos > 2500:
                    xservopos = 2500
                if yservopos > 2500:
                    yservopos = 2500
                
                print (xservopos)
                print (yservopos)
                pi.set_servo_pulsewidth(17, xservopos)
                pi.set_servo_pulsewidth(27, yservopos)
                
                print (time)
                
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

