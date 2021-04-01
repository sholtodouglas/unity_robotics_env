# !/usr/bin/env python


import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService
from robotics_demo.msg import JointPositions, PositionCommand, QuaternionProprioState, FullState, Goal, ResetInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

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
        'state': RosPublisher('state', FullState, queue_size=1),
        # Goals may come from the state, listen if so
        'goal': RosPublisher('goal', Goal, queue_size=1),
        # Set goals in the environment
        'set_goal': RosSubscriber('set_goal', Goal, tcp_server),
        # Get the 'reset' command from the controlller
        'reset': RosPublisher('reset', Bool, queue_size=1),
        # Send a state to reset to
        'reset_state': RosSubscriber('reset_state', ResetInfo, tcp_server),
        #'imageTest': RosPublisher('imageTest', ImageTest, queue_size=1),
    })
    
    rospy.spin()


if __name__ == "__main__":
    main()

