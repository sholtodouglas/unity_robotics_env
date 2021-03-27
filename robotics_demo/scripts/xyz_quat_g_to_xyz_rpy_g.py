#!/usr/bin/env python

import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
os.sys.path.insert(0, currentdir)
import pybullet
import random
import rospy
from robotics_demo.msg import PositionCommand, QuaternionCommand

import numpy as np

# create a publisher to send joint commands out on
pub = rospy.Publisher('xyz_rpy_g_command', PositionCommand, queue_size=10)

def convert2XYZRPY(cmd):
    '''
    Unity quaternion to pybullet RPY equivalent
    '''
    x,y,z,q1,q2,q3,q4,gripper = cmd.pos_x, cmd.pos_y, cmd.pos_z, cmd.q1, cmd.q2, cmd.q3, cmd.q4, cmd.gripper
    rpy = pybullet.getEulerFromQuaternion([-q2, -q1, q4, q3]) # yay for beautiful conversion between axis
    rpy_cmd = PositionCommand(x,y,z,rpy[0], rpy[1], rpy[2], gripper)
    print(rpy)
    pub.publish(rpy_cmd)
   
def listener():
    rospy.init_node('xyz_quat_2_rpy', anonymous=True)
    rospy.Subscriber("xyz_quat_g_command", QuaternionCommand, convert2XYZRPY)
    rospy.spin()

if __name__ == "__main__":
    listener()

