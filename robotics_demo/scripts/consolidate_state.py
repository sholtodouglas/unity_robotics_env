#!/usr/bin/env python



import random
import rospy
from robotics_demo.msg import ArmState
import pybullet


def record_arm_state(state):
    x,y,z,q1,q2,q3,q4,gripper = state.pos_x, state.pos_y, state.pos_z, state.q1, state.q2, state.q3, state.q4, state.gripper
    print(pybullet.getEulerFromQuaternion([-q2, -q1, q4, q3])) # yay for beautiful conversion between axis


def listener():
    rospy.init_node('conslidate_state')
    rospy.Subscriber("state", ArmState, record_arm_state)
    rospy.spin()

if __name__ == "__main__":
    listener()

