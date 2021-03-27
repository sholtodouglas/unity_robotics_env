#!/usr/bin/env python



import random
import rospy
from robotics_demo.msg import ArmState, ImageTest, FullState
import pybullet
from PIL import Image, ImageOps
import os

PACKAGE_LOCATION = os.path.dirname(os.path.realpath(__file__))[:-(len("/scripts"))] # remove "/scripts"

def record_arm_state(state):
    print('hi')
    x,y,z,q1,q2,q3,q4,gripper = state.arm_pos_x, state.arm_pos_y, state.arm_pos_z, state.arm_q1, state.arm_q2, state.arm_q3, state.arm_q4, state.gripper
    print(pybullet.getEulerFromQuaternion([-q2, -q1, q4, q3])) # yay for beautiful conversion between axis

def recordImage(test):
    req = test.shoulderImage
    image_height = req.width
    image_width = req.height
    image = Image.frombytes('RGBA', (image_width,image_height), req.data)
    image = ImageOps.flip(image)
    image.save(PACKAGE_LOCATION+ '/data/test.png')
    


def listener():
    rospy.init_node('conslidate_state')
    rospy.Subscriber("state", FullState, record_arm_state)
    #rospy.Subscriber("imageTest", ImageTest, recordImage)
    rospy.spin()

if __name__ == "__main__":
    listener()

