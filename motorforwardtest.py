import RPi.GPIO as GPIO                    #Import GPIO library
import time
import math
import numpy as np
import sys
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper

kit = MotorKit()
def forward(steps):
   print ("Forward")
   for i in range(steps):
        kit.stepper1.onestep(direction=stepper.FORWARD,style=stepper.DOUBLE)
        kit.stepper2.onestep(direction=stepper.BACKWARD,style=stepper.DOUBLE)
        time.sleep(0.01)
forward(64)