#!/usr/bin/env python3

import rospy

from aut_sys.msg import motors, distance, servos  #importing msg files from specified location
from fiducial_msgs.msg import FiducialTransformArray
import geometry_msgs.msg

from Controlllers import PidController, PidController_2x    #importing our PID controller function

import numpy as np
import random


lost = 1 # this tells us camera can't see marker, that means we should move servos till we can see it. 1 means lost, 0 means not lost
pos_x = 0
pos_z  = 0 


def fud_callback(data):

    global pos_x
    global pos_z
    global lost

    if data.transforms == [] :
        print("Lost")
        lost = 1 # this tells us camera can't see marker, that means we should move servos till we can see it.

    else:
        lost = 0            
        pos_x = trans.transform.translation.x   # setting measured camera distance data to a variable
        pos_z = trans.transform.translation.z
        print(pos_z)

def core():
    global pos_x
    global pos_z
    global lost

    # Create a publisher object
    pub = rospy.Publisher('/motors', motors, queue_size=10) #topic name, msg type, buffer size
    pubs = rospy.Publisher('/servos', servos, queue_size=10) #topic name, msg type, buffer size

    rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, fud_callback) #topic name, msg type, callback function

    # Declare the node, and register it with a unique name
    rospy.init_node('core', anonymous=True)

    rate = rospy.Rate(10) #executing at a rate of 10Hz

    motor_msg = motors() #creating object
    servos_msg = servos() #creating object

    dt = 0.1

    PID_obj_1 = PidController(1.75, 0.05, 0.01, dt, 0.0, 0.8) # set pid values and object to control z position
    PID_obj_2 = PidController(1.55, 0.03, 0.01, dt, 0.0, 0.6) # set pid values and object to control x position

    goal = [1.5,-0.5]  # z, x

    while not rospy.is_shutdown():
        
     
        if (lost == 1): # stop moving if lost
            print(lost)
            motor_msg.rightSpeed = 0
            motor_msg.leftSpeed = 0

        if (lost == 0): 

            err_z = goal[0] - pos_z
            err_x = goal[1] - pos_x

            cmd_Vel_z = PID_obj_1.step(err_z) # step pid function
            cmd_Vel_x = PID_obj_2.step(err_x) # step pid function

            motor_msg.rightSpeed = cmd_Vel_z + cmd_Vel_x    #   speed should be algebraic sum to PID o/p for z and x position
            motor_msg.leftSpeed = cmd_Vel_z - cmd_Vel_x

        pub.publish(motor_msg)
        
        rate.sleep()


if __name__ == '__main__':
    try:
        core()
    except rospy.ROSInterruptException:
        pass