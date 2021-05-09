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
from std_msgs.msg import String
from tqdm import tqdm
import shutil

RECORDING = False
stepCount = 0
# Lets just pipe it direct into LFP
PACKAGE_LOCATION = os.path.dirname(os.path.realpath(__file__))[:-(len("/unity_robotics_env/robotics_demo/scripts"))] + '/learning_from_play/'
base_path = PACKAGE_LOCATION +'/data/envHz25/'
obs_act_path = base_path + 'obs_act_etc/'
env_state_path = base_path + 'states_and_ims/'
example_path = None
npz_path = None

obs_array = []
acts_array = []
ag_array = []
times_array = []
shoulder_imgs = []
gripper_imgs = []

last_tstep_received = 0

RAM_LIMIT = 10000 # How many images its reasonable to store in memory before we get dangerous

saving_status = rospy.Publisher('saving_status', String, queue_size=1)

def save_timestep(timestep: ToRecord):
    '''
    Properties
    o: All state info and the imgs, counter
    a: xyzrpyg action
    data_arrival_time
    model_processed_time
    '''
    global stepCount, last_tstep_received, shoulder_imgs, gripper_imgs
    last_tstep_received = time.time()

    if RECORDING:
        # print(stepCount)
        ag = ag_to_vector(timestep.state.ag)
        arm_state = proprio_rpy_to_rpy_vector(timestep.state.proprio)
        obs_array.append(np.concatenate([arm_state, ag]))
        ag_array.append(ag)

        act = timestep.a 
        acts_array.append(proprio_rpy_to_rpy_vector(timestep.a))

        times_array.append({'timestep': timestep.timestep, 'data_gen_time': timestep.data_gen_time,
                    'data_arrival_time': timestep.data_arrival_time, 'data_processed_time': timestep.data_processed_time, 
                    'beat_sent_time':timestep.beat_sent_time, 'act_begin_time': timestep.act_begin_time, 'model_processed_time': timestep.model_processed_time})
        
        # Append instead of saving because sometimes it was too slow and we missed steps
        shoulder_imgs.append(rosImg_to_numpy(timestep.shoulderImage)) 
        gripper_imgs.append(rosImg_to_numpy(timestep.gripperImage))
        
        stepCount += 1
        print(stepCount)

def start_recording(b: Bool):
    global RECORDING, stepCount, last_tstep_received
    if not RECORDING:
        RECORDING = True
        last_tstep_received = time.time()
        update_filepaths()

        stepCount = 0
        print(f"Started recording at {npz_path}")
        saving_status.publish("Recording")
        ping()

def save_trajectory(x: RPYState):
    
    global RECORDING, obs_array, acts_array, ag_array, times_array, shoulder_imgs, gripper_imgs
    try:
        if RECORDING:
            ping()
            saving_status.publish("Saving")
            RECORDING = False
            np.savez(npz_path + '/data', acts=acts_array, obs=obs_array, achieved_goals=ag_array, times=times_array, allow_pickle=True)
            for i in tqdm(range(0, len(gripper_imgs))):
                plt.imsave(example_path + f'/ims/{i}_shoulder.jpg', shoulder_imgs[i])
                plt.imsave(example_path + f'/ims/{i}_gripper.jpg', gripper_imgs[i])
            obs_array, acts_array, ag_array, times_array, shoulder_imgs, gripper_imgs = [], [], [], [], [], []
            # might do us well to include stuff like controllable achieved goal TODO
            print(f"Recorded {npz_path}")
            saving_status.publish("Not recording")
            ping()
    except Exception as e:
        print(f"failed to record {e}")
        try:
            shutil.rmtree(example_path + '/ims')
            shutil.rmtree(npz_path)
        except Exception as e:
            print(e)

        

def ping():
    try:
        os.system("beep -f 555 -l 460") # throws error, still beeps for it haha!
    except:
        print("This exception is ok its to make it beep")
        pass

    

    

def update_filepaths():
    global example_path, npz_path
    demo_count = len(list(os.listdir(obs_act_path)))
    example_path = env_state_path + str(demo_count)
    npz_path = obs_act_path+str(demo_count)

    os.makedirs(example_path + '/ims')
    os.makedirs(npz_path)


def listener():
    global RECORDING, last_tstep_received, stepCount

    try:
        os.makedirs(env_state_path)
    except Exception as e:
        print(e)
    try:
        os.makedirs(obs_act_path)
    except Exception as e:
        print(e)

    rospy.init_node('conslidate_state')
    rospy.Subscriber("timestep", ToRecord, save_timestep, queue_size=10)
    rospy.Subscriber("start_recording", Bool, start_recording) # state is the commanded info
    rospy.Subscriber("reset", RPYState, save_trajectory) # state is the commanded info
    while not rospy.is_shutdown():
        t = time.time()
        if t > last_tstep_received + 2 or stepCount > RAM_LIMIT: 
            if RECORDING and len(obs_array) > 0:
            #if we are mid recording, but it has been a while since the last timestep - cut off the 
            # trajectory, stop recording until instructed to do so
                print('saving due to connection failure or exceeding RAM limit')
                save_trajectory(RPYState())


if __name__ == "__main__":
    listener()

