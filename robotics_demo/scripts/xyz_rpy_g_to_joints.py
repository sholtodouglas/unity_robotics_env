#!/usr/bin/env python

import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
os.sys.path.insert(0, currentdir)

import random
import rospy
from robotics_demo.msg import PositionCommand, JointPositions
from shadow_arm import InverseKinematicsSolver
import numpy as np

# create a publisher to send joint commands out on
pub = rospy.Publisher('joint_commands', JointPositions, queue_size=10)
IKSolver = InverseKinematicsSolver()

def convert2jointCommand(p_cmd):
    '''
    Data is coming in as unity xyz, but pybullet RPY - yes this is confusing, yes I'm sorry - but unity RPY just doesn't
    appear to be stable enough, so we're using it's quaternions then converting those to pybullet RPY where relevant
    '''
    # Note, negative x to convert from unity coords to pybullet coords
    # print(p_cmd.pos_y)
    x,y,z = -p_cmd.pos_x, p_cmd.pos_y, p_cmd.pos_z
    roll,pitch,yaw = p_cmd.rot_r, p_cmd.rot_p, p_cmd.rot_y  
    gripper = p_cmd.gripper

    j_angs = IKSolver.calc_angles([x,y,z], [roll,pitch,yaw]) # TODO: Feed in current angles for better solutions 
    print(j_angs)
    j_angs = np.array(j_angs) * (180/np.pi) # Unity uses degrees. TODO: Should we convert for this inside the env? TODO: Remember to convet back if reading!
    # print(j_angs)
    cmd = JointPositions(j_angs[0],j_angs[1], j_angs[2], j_angs[3], j_angs[4], j_angs[5], gripper)
    pub.publish(cmd)
   
def listener():
    rospy.init_node('xyz_rpy_g_to_joints', anonymous=True)
    rospy.Subscriber("xyz_rpy_g_command", PositionCommand, convert2jointCommand)
    rospy.spin()

if __name__ == "__main__":
    listener()

