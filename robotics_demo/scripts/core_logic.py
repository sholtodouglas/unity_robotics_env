#!/usr/bin/env python



import random
import rospy
from robotics_demo.msg import Observation, ToRecord, QuaternionProprioState, PositionCommand, Goal, TimerBeat, RPYProprioState, \
                                AchievedGoal, RPYState, ReplayInfo
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import String, Bool
import pybullet
import numpy as np
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print("current_dir=" + currentdir)
os.sys.path.insert(0, currentdir)
from utils import rosImg_to_numpy, proprio_quat_to_rpy_vector, proprio_rpy_to_ROSmsg, ag_to_vector, ag_to_ROSmsg, proprio_rpy_to_rpy_vector, unstack
import np_to_multiarray
import time 

np.set_printoptions(suppress=True)
PACKAGE_LOCATION = os.path.dirname(os.path.realpath(__file__))[:-(len("/scripts"))] # remove "/scripts"

# This variable indicates whether it is being controlled by the env, e.g. through sliders or vr controller
ENV_CONTROLLED = True # It is default false, but set true by the environment on startup

# Stores the latest quaternion command from the VR controller, if active
vr_controller_pos = PositionCommand(-0.4, 0.2, 0.0,0.0,0.0,0.0,0.0)
default_vr_controller_pos =  PositionCommand(-0.4, 0.2, 0.0,0.0,0.0,0.0,0.0)
# LMP stuff
g = None # [1, GOAL_DIM] either state size or img embedding size
replan_horizon = 15
z = None # [1,LATENT_DIM] representing the latent plan
# Store the values which will be updated by the reseprective callbacks for the actor to use
shoulder_image = None
ros_shoulder_image = None
gripper_image = None
ros_gripper_image = None
proprioceptive_state = None
achieved_goal = None
full_state = None
velocities = None
last_state_arrival_time = 0
last_state_processed_time = 0  # unless updated then the if check will fail because 0 is too long ago
last_vr_controller_time = 0

# The publisher which sends out position commands that then get converted to joint commands by the IK node
pos_cmd_pub = rospy.Publisher('xyz_rpy_g_command', PositionCommand, queue_size=1)
# The publisher which sends out the consolidated state for the saver to save
transition_pub = rospy.Publisher('timestep', ToRecord, queue_size=1)
# The publisher which resets the non_arm elements of the state
env_reset_pub = rospy.Publisher('reset_environment', AchievedGoal, queue_size=1)

# resetting flag
resetting = False
# buffer of acts to replay from - if this is full, pop them off one by one!
replay_acts = []

def check_reset_convergence(proprio, ag, threshold = 0.05):
    global proprioceptive_state, achieved_goal
    proprio_checks = abs(proprio - proprioceptive_state) < threshold
    ag_checks = abs(ag-achieved_goal) < threshold
    all_checks = np.concatenate([proprio_checks, ag_checks])

    if np.all(all_checks):
        return True
    else:
        return False


# A function which resets the environment
def reset(state: RPYState):
    resetting = True
    t_reset = time.time()
    pos_cmd_pub.publish(state.proprio)
    env_reset_pub.publish(state.ag)
    # keep looping until too much time has elapsed, or the state is sufficiently close to the desired one
    np_proprio = proprio_rpy_to_rpy_vector(state.proprio)
    np_ag = ag_to_vector(state.ag)
    

    r = rospy.Rate(10) # 10hz 
    while not rospy.is_shutdown():
        t = time.time()
        if (t > t_reset+2):
            print("Reset convegence timed out")
            resetting = False
            return
        elif check_reset_convergence(np_proprio, np_ag):
            print("Reset converged")
            resetting = False
            return
        else:
            pos_cmd_pub.publish(state.proprio)
            env_reset_pub.publish(state.ag)
        r.sleep() # ros sleep for a little while before checking convergence again
            
def replay(replay: ReplayInfo):
    global replay_acts
    acts = np_to_multiarray.to_numpy_f32(replay.acts) # should be T, 7 (ACT_DIM)
    acts = unstack(acts) # will produce a list of acts we can pop from 
    replay_acts += acts
    reset(replay.resetState)



# A series of functions which set the goal dependent on states or goal space
def set_goal(goal: Goal):
    '''
    A function which subscribes to 'img goal', encodes and sends to set goal function
    '''
    global g
    if goal.type == 'state':
        sg = goal.state_goal
        g = np.array([sg.obj1_pos_x, sg.obj1_pos_y, sg.obj1_pos_z, sg.obj1_q1, sg.obj1_q2, sg.obj1_q3, sg.obj1_q4])
    elif goal.type == 'img':
        img_goal = np.expand_dims(rosImg_to_numpy(img_goal)) # [1, H,W,C]
        g = cnn(goal.img_goal)
    elif goal.type == 'lang':
        g = sentence_encoder(goal.lang_goal)
    raise NotImplementedError # not done yet

def register_vr_controller(cmd: QuaternionProprioState):
    '''
    Whenever the VR controller updates, this updates 'vr_controller_pos' so that act can send that as the action
    instead of model outputs if a VR controller is plugged in 
    '''
    global vr_controller_pos, last_vr_controller_time
    x,y,z,q1,q2,q3,q4,gripper = cmd.pos_x, cmd.pos_y, cmd.pos_z, cmd.q1, cmd.q2, cmd.q3, cmd.q4, cmd.gripper
    rpy = pybullet.getEulerFromQuaternion([-q2, -q1, q4, q3]) # yay for beautiful conversion between axis
    #print(x,y,z, rpy)
    vr_controller_pos = PositionCommand(x,y,z,rpy[0], rpy[1], rpy[2], gripper)
    last_vr_controller_time = time.time()

def process_observation(o: Observation):
    '''
    The full state will be sent out at Nh > controlHz by the env, listen to it, and save the relevant parts
    '''
    global shoulder_image, gripper_image, proprioceptive_state, \
            achieved_goal, full_state, last_state_arrival_time, \
            last_state_processed_time, ros_shoulder_image, ros_gripper_image, velocities
    last_state_arrival_time = time.time()
    
    ros_shoulder_image = o.shoulderImage
    shoulder_image  = rosImg_to_numpy(ros_shoulder_image)
    ros_gripper_image = o.gripperImage
    gripper_image = rosImg_to_numpy(ros_gripper_image)

    proprioceptive_state  = proprio_quat_to_rpy_vector(o.proprio)
    # print(f"State: {proprioceptive_state}")
    achieved_goal = ag_to_vector(o.ag)
    velocities = o.vels
    full_state = np.concatenate([proprioceptive_state, achieved_goal])
    last_state_processed_time = time.time()


def act(b: TimerBeat):
    '''
    Subscribes to 'beat' topic, which occurs when the timer loop hits time
    Ouputs 
    a) an xyzrpygripper act on topic 'xyz_rpy_g_command', which gets converted to joint angles by 'xyz_rpy_g_to_joints.py'
    b) a combined state + act on topic 'transition', for the recorder to record
    '''
    global  g,z, vr_controller_pos, shoulder_image, gripper_image, proprioceptive_state, full_state, velocities

    #print([x,y,z])
    #print(pybullet.getEulerFromQuaternion([-q2, -q1, q4, q3])) # at rest, should be 0,0,0

    act_begin_time = time.time()

    if act_begin_time-last_state_processed_time > 0.5:
        print("No recent env information - check connection")
        return 
    elif resetting:
        #print("Resetting - no action take")
        return
    else:
        
        #### Put these into the model ####
        # if o.timestep % replan_horizon == 0:
        #      actor.reset_states()
        #     z = planner((obs, g))
        # a = model((obs, z, g))
        # a = PositionCommand(a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7])
        model_processed_time = time.time()

        # Publish the action for the environment to take
        if ENV_CONTROLLED:
            if time.time() > last_vr_controller_time + 2:
                a = default_vr_controller_pos
                
            else:
                a = vr_controller_pos
        elif len(replay_acts) > 0: # if there are acts to replay, pop them off instead of taking our own
            a = replay_acts.pop()
        
        pos_cmd_pub.publish(a)

        # Consolidate o and a, include when it arrived, and how long model processing took, o has the initial request time
        proprio = proprio_rpy_to_ROSmsg(proprioceptive_state)
        ag = ag_to_ROSmsg(achieved_goal)
        state = RPYState(proprio, ag)
        timestep = ToRecord(state, velocities, ros_shoulder_image, ros_gripper_image, a, b.timestep,\
            last_state_arrival_time, last_state_processed_time,  b.time, act_begin_time, model_processed_time)
        # # and publish this for the saver to record
        transition_pub.publish(timestep) 



def listener():
    rospy.init_node('core_logic')
    # Listens for states, sends out an xyzrpy action in response
    rospy.Subscriber("state", Observation, process_observation)
    rospy.Subscriber("beat", TimerBeat, act)
    # Listens for the VR controller pos, updates the variable to store it
    rospy.Subscriber("xyz_quat_g_command", QuaternionProprioState, register_vr_controller)
    rospy.Subscriber("goal", Goal, set_goal)
    rospy.Subscriber("reset", RPYState, reset) # state is the commanded info
    rospy.Subscriber("replay", ReplayInfo, replay)

    rospy.spin()


if __name__ == "__main__":
    listener()





# We need to create a main loop, which emits 'get obs' every 1/30th of a second
# The env listens for this, and emits an 'obs' in response
# There is a model node, which constantly listens for obs, and outputs a) an act, and b) a paired example (to eliminate time synch issues)
# There is a planner node, which similarly listens for 'replan, which includes a goal' and similarly outputs a new z, 
#       which triggers an lstm restart in the actor node
# There is a saver node, which takes these paired examples, and saves them for training

# There is a reset button on the gripper, which triggers the saver node to save its buffers
# ------------------------------------------------------------------------------------------
# Initially, everything in one main loop
# Every 20th, ping the env requesting obs with an index indicating the t_step, and a request time
# has a subscriber to obs (now with time included) which outputs act, if its at a replan interval then we spend very slightly longer running a plan to
#       if we notice this decreases performance, we can shift this to the end of the Nth timestep, act has a timestep and a time
# Also have a subscriber to the controller constantly updating, 'model' outputs this instead. Idea - literally have the model running in there so the timing is identical?

# have a separate saver node, the obs and act both should have the same timestep, then get saved together. 

# Check time required to remit the msg, if tiny then just do that and ensure good timestep health

# How to replay? Send a 'reset' service call, arm converges into position - similarly request obs, take action from replay buffer
