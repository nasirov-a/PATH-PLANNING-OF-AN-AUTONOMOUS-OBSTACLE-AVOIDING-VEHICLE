import RPi.GPIO as GPIO                    #Import GPIO library
import time
import math
import numpy as np
import sys

# from movetopose import move_to_pose
from simpletest import bno
from Potential import mainplan
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
from random import random


kit = MotorKit()

#Import time library
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)  
startangle=0

xobs=0
yobs=0
direction=0
count=0
step = 16  # 1 cm
ministep=48 # 1.7171052631578947 degree angle checked by bn055 [1.8125, 1.75, 1.75, 1.625, 1.625, 1.8125, 1.6875, 1.6875, 1.6875, 1.6875, 1.6875, 1.6875, 1.625, 1.8125, 1.75, 1.6875, 1.8125, 1.6875, 1.75]
#Pinout

TRIGfront = 6
ECHOfront = 5

TRIGback = 23
ECHOback = 24

TRIGright = 16
ECHOright = 22

TRIGleft = 27
ECHOleft = 17


GPIO.setup(TRIGfront,GPIO.OUT)                  # initialize GPIO Pin as outputs
GPIO.setup(ECHOfront,GPIO.IN)                   # initialize GPIO Pin as input

GPIO.setup(TRIGback,GPIO.OUT)                  # initialize GPIO Pin as outputs
GPIO.setup(ECHOback,GPIO.IN)  

GPIO.setup(TRIGright,GPIO.OUT)                  # initialize GPIO Pin as outputs
GPIO.setup(ECHOright,GPIO.IN)  

GPIO.setup(TRIGleft,GPIO.OUT)                  # initialize GPIO Pin as outputs
GPIO.setup(ECHOleft,GPIO.IN)  

avgDistance=0


# Motor Codes




def right(steps):
   """ Clockwise """
   print ("Right")
   for i in range(steps):
       kit.stepper1.onestep(direction=stepper.FORWARD,style=stepper.MICROSTEP)
       kit.stepper2.onestep(direction=stepper.FORWARD,style=stepper.MICROSTEP)
    #    time.sleep(0.1)
       
       

def left(steps):    
    print ("Left")
    for i in range(steps):
        kit.stepper1.onestep(direction=stepper.BACKWARD,style=stepper.MICROSTEP)
        kit.stepper2.onestep(direction=stepper.BACKWARD,style=stepper.MICROSTEP)
        # time.sleep(0.1)
        

def back(steps):
   print ("Back")
   for i in range(steps):
       kit.stepper1.onestep(direction=stepper.BACKWARD,style=stepper.DOUBLE)
       kit.stepper2.onestep(direction=stepper.FORWARD,style=stepper.DOUBLE)
       time.sleep(0.01)
        
  
    

def forward(steps):
   print ("Forward")
   for i in range(steps):
        kit.stepper1.onestep(direction=stepper.FORWARD,style=stepper.DOUBLE)
        kit.stepper2.onestep(direction=stepper.BACKWARD,style=stepper.DOUBLE)
        time.sleep(0.01)
        



def moveCar(x,y,finalangle,startangle,epsilon = 1):  
    if finalangle < 0:
        calctheta=-finalangle
        direction=1 #right
        
    else:
        calctheta=360-finalangle
        direction=0 #left
   
   
    heading = bno.read_euler()
    angle=heading[0]
    if(angle < calctheta): 
        if(abs(angle - calctheta)<180):
            difference=abs(angle - calctheta)
            multiplier=difference/1.7171052631578947
            right(int(ministep*multiplier))
        else:
            difference= 360 - abs(angle - calctheta)
            multiplier=difference/1.7171052631578947
            left(int(ministep*multiplier))
            
    else: 
        if(abs(angle - calctheta)<180):
            difference=abs(angle - calctheta)
            multiplier=difference/1.7171052631578947
            left(int(ministep*multiplier))
        else:
            difference=360 -abs(angle - calctheta)
            multiplier=difference/1.7171052631578947
            right(int(ministep*multiplier))
            
    print("difference angle %s and multiplier %s"%(difference,multiplier))  
    dis = int(np.hypot(x, y)) # Round
    print("Calculated Distance %s from hypot X %s and Y %s"%(dis,x,y))
    forward(dis*step)
    print(dis*step)




def distance(tripin,echpin,direction): 
    GPIO.output(tripin, True)    
    #  set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(tripin, False)
    StartTime = time.time()
    StopTime = time.time()
    
    #  save StartTime
    while GPIO.input(echpin) == 0:
        StartTime = time.time()
    
    #  save time of arrival
    while GPIO.input(echpin) == 1:
        StopTime = time.time()
    
    #  time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    
    #  multiply with the sonic speed (34300 cm/s)
    #  and divide by 2, because there and back
    avgDistance = (TimeElapsed * 34300) / 2
    print(str(avgDistance) + "    " + direction)
    return avgDistance

import math

def ocoordinates(xs,ys,d,alpha):
    
    xobs = math.sqrt(d**2-(d**2/(1+1/math.degrees(math.tan(alpha)**2))))+xs
    yobs = math.sqrt(d**2/(1+1/math.degrees(math.tan(alpha)**math.tan(alpha))))+ys
    return(xobs.real,yobs.real)







def main():

    while True:
        x_start = 0
        y_start = 0
        theta_goal = 0
        oy=[6.8,12.6,9.7] #static obstacles at the y coordinate
        ox=[7.8,12.6,20.4] #static obstacle at the x coordinate
        global startangle
        
        heading = bno.read_euler()
        startangle=heading[0] # angle measure by BNO005

        X,Y = mainplan(gx=27.3,gx=19.0,ox=ox,oy=oy)
        
        time.sleep(1)    
        for ind,x in enumerate(X):
            print("X coordinates %s and Y coordinates %s"%(x*10,Y[ind]*10))
            theta_goal=math.degrees(math.atan2(Y[ind]*10-y_start, x*10-x_start)) # calculating the required angle
            print("Theta calculated from the vectors theta=%s"%(theta_goal))
            moveCar(x*10,Y[ind]*10,theta_goal,startangle) # moving a vehicle to the required position
            distance(TRIGfront,ECHOfront)
            if distance<50:
                ocoordinates(x_start,x_start,avgDistance,startangle)
                ox.append(xobs.real)
                oy.append(yobs.real)
                break
            else:
                pass
            x_start=x*10 
            y_start=Y[ind]*10
        

main()





