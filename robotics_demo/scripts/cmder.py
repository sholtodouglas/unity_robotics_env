#!/usr/bin/env python

import random
import rospy
from robotics_demo.msg import JointPositions


TOPIC_NAME = 'joint_commands'
NODE_NAME = 'joint_publisher'


def post_color():
    pub = rospy.Publisher(TOPIC_NAME, JointPositions, queue_size=10)
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        shoulder = 30
        upper_arm = 30
        forearm  = 30
        wrist_1 = 30
        wrist_2 = 30
        wrist_3 = 30
        gripper = 30
        
        cmd = JointPositions(shoulder,upper_arm, forearm, wrist_1, wrist_2, wrist_3, gripper)
        pub.publish(cmd)
        rate.sleep()
        break


if __name__ == '__main__':
    try:
        post_color()
    except rospy.ROSInterruptException:
        pass
