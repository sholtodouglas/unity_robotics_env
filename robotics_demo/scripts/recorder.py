#!/usr/bin/env python



import random
import rospy
from robotics_demo.msg import ToRecord, RPYState
import pybullet
from PIL import Image, ImageOps
import os
import time 
import numpy as np
import matplotlib.pyplot as plt
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print("current_dir=" + currentdir)
os.sys.path.insert(0, currentdir)
from utils import rosImg_to_numpy, proprio_quat_to_rpy_vector, proprio_rpy_to_ROSmsg, ag_to_vector, ag_to_ROSmsg, proprio_rpy_to_rpy_vector

DEBUGGING = False

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
    
    ag = ag_to_vector(timestep.state.ag)
    arm_state = proprio_rpy_to_rpy_vector(timestep.state.proprio)
    obs_array.append(np.concatenate([arm_state, ag]))
    ag_array.append(ag)

    act = timestep.a 
    acts_array.append(proprio_rpy_to_rpy_vector(timestep.a))

    times_array.append({'timestep': timestep.timestep,
                'data_arrival_time': timestep.data_arrival_time, 'model_processed_time': timestep.model_processed_time, 
                'beat_sent_time':timestep.beat_sent_time, 'act_begin_time': timestep.act_begin_time, 'model_processed_time': timestep.model_processed_time})
    
    if not DEBUGGING:
        plt.imsave(example_path + f'/env_images/{timestep.timestep}_shoulder.jpg', rosImg_to_numpy(timestep.shoulderImage))
        plt.imsave(example_path + f'/env_images/{timestep.timestep}_gripper.jpg', rosImg_to_numpy(timestep.gripperImage))

def save_trajectory(x: RPYState):
    global obs_array, acts_array, ag_array, times_array
    if not DEBUGGING:
        np.savez(npz_path + '/data', acts=acts_array, obs=obs_array, achieved_goals=ag_array, times=times_array, allow_pickle=True)
        obs_array, acts_array, ag_array, times_array = [], [], [], []
        # might do us well to include stuff like controllable achieved goal TODO

    update_filepaths()

def update_filepaths():
    global example_path, npz_path
    demo_count = len(list(os.listdir(obs_act_path)))
    example_path = env_state_path + str(demo_count)
    npz_path = obs_act_path+str(demo_count)
    if not DEBUGGING:
        os.makedirs(example_path + '/env_states')
        os.makedirs(example_path + '/env_images')
        os.makedirs(npz_path)


def listener():

    try:
        os.makedirs(env_state_path)
        os.makedirs(obs_act_path)
    except:
        pass
    update_filepaths()
    
    rospy.init_node('conslidate_state')
    rospy.Subscriber("timestep", ToRecord, save_timestep)
    rospy.Subscriber("reset", RPYState, save_trajectory) # state is the commanded info
    rospy.spin()


if __name__ == "__main__":
    listener()

