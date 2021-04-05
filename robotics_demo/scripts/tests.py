import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
os.sys.path.insert(0, currentdir)

import random
import rospy
import numpy as np
import np_to_multiarray
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import time
# create a publisher to send joint commands out on
pub = rospy.Publisher('test', Float32MultiArray, queue_size=1)

CONTROL_FREQUENCY = 2

def test(t):
    '''
    Data is coming in as unity xyz, but pybullet RPY - yes this is confusing, yes I'm sorry - but unity RPY just doesn't
    appear to be stable enough, so we're using it's quaternions then converting those to pybullet RPY where relevant
    '''
    print(np_to_multiarray.to_numpy_f32(t))

    
   
def listener():
    rospy.init_node('xyz_rpy_g_to_joints', anonymous=True)
    rospy.Subscriber("test", Float32MultiArray, test)
    t0 = time.time()
    next_time = t0 + 1/CONTROL_FREQUENCY
    while not rospy.is_shutdown():
        t = time.time()
        if t >= next_time:
            # When we hit 25Hz (or whatever), send out a request for an obs, which trigger corelogic sending an act back
            
            try:
                next_time = next_time + 1/CONTROL_FREQUENCY
                data = np.ones([4,8])
                data = np_to_multiarray.to_multiarray_f32(data)
                pub.publish(data)
                
            except:
                return 

if __name__ == "__main__":
    listener()

