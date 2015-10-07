#!/usr/bin/env python
import rospy
import time
import atexit
import threading
import random

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
from waterbot_msgs.msg import stepper

# Instantiate motor drivers class #
mh = Adafruit_MotorHAT()
# Setup motor 1 driver to 200 steps/rev #
myStepper1 = mh.getStepper(200, 2)
# Setup speed to 100 #
myStepper1.setSpeed(100)
# Define kinds of steps #
stepstyles = [Adafruit_MotorHAT.SINGLE, Adafruit_MotorHAT.DOUBLE, Adafruit_MotorHAT.INTERLEAVE, Adafruit_MotorHAT.MICROSTEP]

# Turn off motor signals #
def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

# Stepper 2 movement topic call back #
def callback(data):
    if (data.forward):
        # Move data.steps amount of steps 
        # In the forward direction
        # Double steps (Adafruit_MotorHAT.DOUBLE)
        myStepper1.step(data.steps, Adafruit_MotorHAT.FORWARD, 2)
    else:    
        # Move data.steps amount of steps 
        # In the backward direction
        # Double steps (Adafruit_MotorHAT.DOUBLE) 
        myStepper1.step(data.steps, Adafruit_MotorHAT.BACKWARD, 2)
    # Turn off motors
    turnOffMotors()

def listener():

    # Initialize stepper 1 node #
    rospy.init_node('stepper2', anonymous=True)

    # Listen to stepper1 movement topic #
    rospy.Subscriber("/stepper2/movement", stepper, callback)

    # Let node run indefinitely, till a kill signal (Ctrl-c or master node ends) #
    rospy.spin()

if __name__ == '__main__':
    listener()
