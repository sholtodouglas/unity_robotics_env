import rospy
from sensor_msgs.msg import Image as ImageMsg
from robotics_demo.msg import Observation, ToRecord, QuaternionProprioState, PositionCommand, Goal, TimerBeat,\
     RPYProprioState, AchievedGoal
from PIL import Image, ImageOps
import numpy as np
import pybullet

def rosImg_to_numpy(img: ImageMsg):
    image_height = img.width
    image_width = img.height
    image = Image.frombytes('RGBA', (image_width,image_height), img.data)
    image = ImageOps.flip(image)
    return np.array(image)


def proprio_quat_to_rpy_vector(o: QuaternionProprioState):
    x,y,z,q1,q2,q3,q4,gripper = o.pos_x, o.pos_y, o.pos_z, o.q1, o.q2, o.q3, o.q4, o.gripper
    rpy = pybullet.getEulerFromQuaternion([-q2, -q1, q4, q3]) # yay for beautiful conversion between axis
    return np.array([x,y,z,rpy[0], rpy[1], rpy[2], gripper])

def proprio_rpy_to_rpy_vector(o: RPYProprioState):
    return np.array([o.pos_x, o.pos_y, o.pos_z, o.rot_r, o.rot_p, o.rot_y, o.gripper])

def proprio_rpy_to_ROSmsg(o: np.ndarray):
    return RPYProprioState(o[0],o[1], o[2], o[3], o[4], o[5], o[6])

def ag_to_vector(o: AchievedGoal):
    return np.array([o.obj1_pos_x, o.obj1_pos_y, o.obj1_pos_z, o.obj1_q1, o.obj1_q2, o.obj1_q3, o.obj1_q4])

def ag_to_ROSmsg(o: np.ndarray):
    return AchievedGoal(o[0],o[1], o[2], o[3], o[4], o[5], o[6])

def unstack(a, axis = 0):
    return [np.take(a, i, axis = axis) for i in range(a.shape[axis])]