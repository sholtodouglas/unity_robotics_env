import rospy
from sensor_msgs.msg import Image as ImageMsg
from robotics_demo.msg import FullState, ToRecord, QuaternionCommand, PositionCommand, Goal, TimerBeat, ProprioceptiveState, AchievedGoal, ResetInfo
from PIL import Image, ImageOps
import numpy as np

def rosImg_to_numpy(img: ImageMsg):
    image_height = img.width
    image_width = img.height
    image = Image.frombytes('RGBA', (image_width,image_height), img.data)
    image = ImageOps.flip(image)
    return np.array(image)


def proprio_to_vector(o: ProprioceptiveState):
    return np.array([o.arm_pos_x, o.arm_pos_y, o.arm_pos_z, o.arm_q1, o.arm_q2, o.arm_q3, o.arm_q4, o.gripper])

def proprio_to_ROSmsg(o: np.ndarray):
    return ProprioceptiveState(o[0],o[1], o[2], o[3], o[4], o[5], o[6])

def ag_to_vector(o: AchievedGoal):
    return np.array([o.obj1_pos_x, o.obj1_pos_y, o.obj1_pos_z, o.obj1_q1, o.obj1_q2, o.obj1_q3, o.obj1_q4])

def ag_to_ROSmsg(o: np.ndarray):
    return AchievedGoal(o[0],o[1], o[2], o[3], o[4], o[5], o[6])
