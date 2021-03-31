#!/usr/bin/env python



import random
import rospy
import os
import time 
from robotics_demo.msg import ObsRequest

PACKAGE_LOCATION = os.path.dirname(os.path.realpath(__file__))[:-(len("/scripts"))] # remove "/scripts"

CONTROL_FREQUENCY = 15 # Hz
timestep = 0 
# Request an obs from the environment at a set time, which results in an action being sent back by corelogic
get_obs_pub = rospy.Publisher('beat', ObsRequest, queue_size=1)

def listener():
    global timestep
    rospy.init_node('timerloop')

    t0 = time.time()
    next_time = t0 + 1/CONTROL_FREQUENCY
    while not rospy.is_shutdown():
        t = time.time()
        if t >= next_time:
            # When we hit 25Hz (or whatever), send out a request for an obs, which trigger corelogic sending an act back
            
            try:
                next_time = next_time + 1/CONTROL_FREQUENCY
                print(timestep, t)
                get_obs_pub.publish(ObsRequest(timestep, t))
                timestep += 1
            except:
                return 


if __name__ == "__main__":
    listener()

    

# This could also be run from jupyter notebook with just the functions of corelogic, which is why its separated