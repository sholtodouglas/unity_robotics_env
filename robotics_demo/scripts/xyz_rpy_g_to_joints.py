#!/usr/bin/env python

import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
os.sys.path.insert(0, currentdir)

import random
import rospy
from robotics_demo.msg import PositionCommand, JointPositions, RPYState
from robotics_demo.srv import getIK, getIKResponse
from shadow_arm import InverseKinematicsSolver
import numpy as np
import time
# create a publisher to send joint commands out on
pub = rospy.Publisher('joint_commands', JointPositions, queue_size=10)
IKSolver = InverseKinematicsSolver()
time_of_last_solve = time.time()

'''
IK_service is a service call to get IK
convert2jointCommand automatically rebroadcasts any XYZRPY commands as joint commands
'''

def IK(p_cmd):
    x,y,z = -p_cmd.pos_x, p_cmd.pos_y, p_cmd.pos_z
    roll,pitch,yaw = p_cmd.rot_r, p_cmd.rot_p, p_cmd.rot_y  
    gripper = p_cmd.gripper

    j_angs = IKSolver.calc_angles([x,y,z], [roll,pitch,yaw]) # TODO: Feed in current angles for better solutions 
    #print(j_angs)
    j_angs = np.array(j_angs) * (180/np.pi) # Unity uses degrees. TODO: Should we convert for this inside the env? TODO: Remember to convet back if reading!
    # print(j_angs)
    cmd = JointPositions(j_angs[0],j_angs[1], j_angs[2], j_angs[3], j_angs[4], j_angs[5], gripper, time.time())
    return cmd


def convert2jointCommand(p_cmd):
    '''
    Data is coming in as unity xyz, but pybullet RPY - yes this is confusing, yes I'm sorry - but unity RPY just doesn't
    appear to be stable enough, so we're using it's quaternions then converting those to pybullet RPY where relevant
    '''
    # Note, negative x to convert from unity coords to pybullet coords
    # print(p_cmd.pos_y)
    global time_of_last_solve
    global IKSolver
    
    if time.time() > time_of_last_solve + 3: # If we haven't solved in 3s, likely the env has been reset - reset the arm too!
        IKSolver = InverseKinematicsSolver()

    cmd = IK(p_cmd)
    pub.publish(cmd)
    time_of_last_solve = time.time() # reset the timer since last solve

def IK_Service(req):
    '''
    Sometimes we want to get the angles for our own use, rather than broadcasting
    Specifically - when resetting
    '''
    global time_of_last_solve
    global IKSolver
    if time.time() > time_of_last_solve + 3: # If we haven't solved in 3s, likely the env has been reset - reset the arm too!
        IKSolver = InverseKinematicsSolver()

    prev = req.prev_joints # the actually commanded joint angles, so we can reset the IK init close to them
    poses = np.array([0.0, prev.shoulder, prev.upper_arm, prev.forearm, prev.wrist_1, prev.wrist_2, prev.wrist_3, 0.0])
    if (poses != 0).any():
        # Then we have actually set desired prev angles TODO: For some reason this hurts rather than helps?
        IKSolver.set_states(poses / (180/np.pi) )
        j = IK(req.RPYPos) # call it once here for convergence
    # else Its generic, don't reset to these
    j = IK(req.RPYPos) # call it here to get the real answer
    time_of_last_solve = time.time() # reset the timer since last solve
    return getIKResponse(j)


def reset(r):
    IKSolver.reset()
   
def listener():
    rospy.init_node('xyz_rpy_g_to_joints', anonymous=True)
    rospy.Subscriber("xyz_rpy_g_command", PositionCommand, convert2jointCommand)
    rospy.Subscriber("reset", RPYState, reset) # state is the commanded info
    rospy.Service('get_IK', getIK, IK_Service)
    rospy.spin()

if __name__ == "__main__":
    listener()

