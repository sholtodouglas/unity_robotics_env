#!/usr/bin/env python



import random
import rospy
from robotics_demo.msg import ToRecord
from std_mgs.msg import Bool
import pybullet
from PIL import Image, ImageOps
import os
import time 
import numpy as np
import matplotlib.pyplot as plt

DEBUGGING = True 

PACKAGE_LOCATION = os.path.dirname(os.path.realpath(__file__))[:-(len("/scripts"))] # remove "/scripts"
base_path = PACKAGE_LOCATION +'/data/UR5/'
obs_act_path = base_path + 'obs_act_etc/'
env_state_path = base_path + 'states_and_ims/'
example_path = None
npz_path = None

obs_array = []
acts_array = []
ag_array = []
times_array = []



def save_timestep(timestep: ToRecord):
    '''
    Properties
    o: All state info and the imgs, counter
    a: xyzrpyg action
    data_arrival_time
    model_processed_time
    '''
    obs = timestep.o
    ag = np.array([obs.obj1_pos_x, obs.obj1_pos_y, obs.obj1_pos_z, obs.obj1_q1, obs.obj1_q2, obs.obj1_q3, obs.obj1_q4])
    arm_state = np.array([obs.arm_pos_x, obs.arm_pos_y, obs.arm_pos_z, obs.arm_q1, obs.arm_q2, obs.arm_q3, obs.arm_q4, obs.gripper])
    obs_array.append(np.concatentate([arm_state, ag]))
    ag_array.append(ag)

    act = timestep.a 
    acts_array.append(np.array([act.pos_x, act.pos_y, act.pos_z, act.rot_r, act.rot_p, act.rot_y, act.gripper]))

    times_array.append({'timestep': obs.timestep, 'request_time': obs.request_time, 
                'data_arrival_time': timestep.data_arrival_time, 'model_processed_time': timestep.model_processed_time})
    
    
    plt.imsave(PACKAGE_LOCATION+ example_path + f'/env_images/{obs.timestep}_shoulder.jpg', rosImg_to_numpy(obs.shoulderImage))
    plt.imsave(PACKAGE_LOCATION+ example_path + f'/env_images/{obs.timestep}_gripper.jpg', rosImg_to_numpy(obs.gripperImage))

def save_trajectory(x: Bool):
    if not DEBUGGING:
        np.savez(npz_path + '/data', acts=acts_array, obs=obs_array, achieved_goals=ag_array, times=times_array, allow_pickle=True)
        obs_array, acts_array, ag_array, times_array = [], [], [], []
        # might do us well to include stuff like controllable achieved goal

    update_filepaths()

def update_filepaths():
    global example_path
    global npz_path 
    demo_count = len(list(os.listdir(obs_act_path)))
    example_path = env_state_path + str(demo_count)
    npz_path = obs_act_path+str(demo_count)


def listener():
    update_filepaths()

    if not DEBUGGING:
        os.makedirs(example_path + '/env_states')
        os.makedirs(example_path + '/env_images')
        os.makedirs(npz_path)

    

    rospy.init_node('conslidate_state')
    rospy.Subscriber("state", ToRecord, save_timestep)
    rospy.Subscriber("reset", Bool, save_trajectory)
    #rospy.Subscriber("imageTest", ImageTest, recordImage)
    rospy.spin()


if __name__ == "__main__":
    listener()

