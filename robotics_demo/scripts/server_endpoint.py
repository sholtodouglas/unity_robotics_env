# !/usr/bin/env python


import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService, UnityService
from robotics_demo.msg import JointPositions, PositionCommand, QuaternionProprioState, Observation, Goal, RPYState, ResetInfo, Reengage
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from robotics_demo.srv import getState

def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    buffer_size = rospy.get_param("/TCP_BUFFER_SIZE", 30000)
    connections = rospy.get_param("/TCP_CONNECTIONS", 20)
    tcp_server = TcpServer(ros_node_name, buffer_size, connections)
    rospy.init_node(ros_node_name, anonymous=True)
    # Fix the subscribe thigns
    tcp_server.start({
        # Publishers and subscribers are w.r.t Unity
        # Allow the robot's sliders to also publish joint commands back to itself
        'joint_commands': RosPublisher('joint_commands', JointPositions, queue_size=1),
        # Publish joint commands to the robot to make it move
        'joint_publisher': RosSubscriber('joint_commands', JointPositions, tcp_server),
        # Subscribe to xyz_rpy_grip commands (these will come from AI or sliders) then rebroadcast joint_commands
        'xyz_rpy_g_command': RosPublisher('xyz_rpy_g_command', PositionCommand, queue_size=1),
        # Subscribe to xyz_quat_grip commands (these will come from the vr controller),store in core logic
        'xyz_quat_g_command': RosPublisher('xyz_quat_g_command', QuaternionProprioState, queue_size=1),
        # The state
        'state': RosPublisher('state', Observation, queue_size=1),
        # Goals may come from the state, listen if so
        'goal': RosPublisher('goal', Goal, queue_size=1),
        # Set goals in the environment
        'set_goal': RosSubscriber('set_goal', Goal, tcp_server),
        # Get the 'reset' command from the controller
        'reset': RosPublisher('reset', RPYState, queue_size=1),
        'start_recording': RosPublisher('start_recording', Bool, queue_size=1),
        # Allows us to push environment updates to the robot
        'full_reset': RosSubscriber('full_reset', ResetInfo, tcp_server),
        're_engage_physics': RosSubscriber("re_engage_physics", Reengage, tcp_server),
        're_engage_collision': RosSubscriber("re_engage_collision", Reengage, tcp_server),

        'getState': UnityService('getState', getState, tcp_server)
        #'imageTest': RosPublisher('imageTest', ImageTest, queue_size=1),
        # getting the exact joint angles is highly fraught (clearly unity immature for robotics), so would rather get exact pose and return required IK
        # which is much more accurate
    })
    
    rospy.spin()


if __name__ == "__main__":
    main()

