#!/usr/bin/env python



import random
import rospy
from robotics_demo.msg import ToRecord, RPYState
from std_msgs.msg import Bool
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

RECORDING = False
stepCount = 0
PACKAGE_LOCATION = os.path.dirname(os.path.realpath(__file__))[:-(len("/unity_robotics_env/robotics_demo/scripts"))] # remove "/scripts"
base_path = PACKAGE_LOCATION +'/data/UR5/'
obs_act_path = base_path + 'obs_act_etc/'
env_state_path = base_path + 'states_and_ims/'
example_path = None
npz_path = None

obs_array = []
acts_array = []
ag_array = []
times_array = []

last_tstep_received = 0

def save_timestep(timestep: ToRecord):
    '''
    Properties
    o: All state info and the imgs, counter
    a: xyzrpyg action
    data_arrival_time
    model_processed_time
    '''
    global stepCount

    if RECORDING:
        ag = ag_to_vector(timestep.state.ag)
        arm_state = proprio_rpy_to_rpy_vector(timestep.state.proprio)
        obs_array.append(np.concatenate([arm_state, ag]))
        ag_array.append(ag)

        act = timestep.a 
        acts_array.append(proprio_rpy_to_rpy_vector(timestep.a))

        times_array.append({'timestep': timestep.timestep,
                    'data_arrival_time': timestep.data_arrival_time, 'model_processed_time': timestep.model_processed_time, 
                    'beat_sent_time':timestep.beat_sent_time, 'act_begin_time': timestep.act_begin_time, 'model_processed_time': timestep.model_processed_time})
        
        
        plt.imsave(example_path + f'/env_images/{stepCount}_shoulder.jpg', rosImg_to_numpy(timestep.shoulderImage))
        plt.imsave(example_path + f'/env_images/{stepCount}_gripper.jpg', rosImg_to_numpy(timestep.gripperImage))
        stepCount += 1

def start_recording(b: Bool):
    global RECORDING, stepCount
    update_filepaths()
    RECORDING = True
    stepCount = 0
    print(f"Started recording at {npz_path}")

def save_trajectory(x: RPYState):
    global RECORDING, obs_array, acts_array, ag_array, times_array
    if RECORDING:
        np.savez(npz_path + '/data', acts=acts_array, obs=obs_array, achieved_goals=ag_array, times=times_array, allow_pickle=True)
        obs_array, acts_array, ag_array, times_array = [], [], [], []
        # might do us well to include stuff like controllable achieved goal TODO
        print(f"Recorded {npz_path}")
    RECORDING = False

    

def update_filepaths():
    global example_path, npz_path
    demo_count = len(list(os.listdir(obs_act_path)))
    example_path = env_state_path + str(demo_count)
    npz_path = obs_act_path+str(demo_count)
    if RECORDING:
        os.makedirs(example_path + '/env_states')
        os.makedirs(example_path + '/env_images')
        os.makedirs(npz_path)


def listener():
    global RECORDING

    try:
        os.makedirs(env_state_path)
    except:
        pass
    try:
        os.makedirs(obs_act_path)
    except:
        pass

    rospy.init_node('conslidate_state')
    rospy.Subscriber("timestep", ToRecord, save_timestep)
    rospy.Subscriber("start_recording", Bool, start_recording) # state is the commanded info
    rospy.Subscriber("reset", RPYState, save_trajectory) # state is the commanded info
    while not rospy.is_shutdown():
        t = time.time()
        if t > last_tstep_received + 2 and RECORDING and len(obs_array) > 0:
            #if we are mid recording, but it has been a while since the last timestep - cut off the 
            # trajectory, stop recording until instructed to do so
            save_trajectory(RPYState())
            RECORDING = False


if __name__ == "__main__":
    listener()

