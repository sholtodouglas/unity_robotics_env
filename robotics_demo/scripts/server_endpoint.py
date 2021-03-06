# !/usr/bin/env python


import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService
from robotics_demo.msg import ArmState, PosRot, UnityColor, JointPositions, PositionCommand, QuaternionCommand
from robotics_demo.srv import PositionService

def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    buffer_size = rospy.get_param("/TCP_BUFFER_SIZE", 1024)
    connections = rospy.get_param("/TCP_CONNECTIONS", 10)
    tcp_server = TcpServer(ros_node_name, buffer_size, connections)
    rospy.init_node(ros_node_name, anonymous=True)
    # Fix the subscribe thigns
    tcp_server.start({
        # Publishers and subscribers are w.r.t Unity
        'pos_srv': RosService('position_service', PositionService),
        # Allow the robot's sliders to also publish joint commands
        'joint_commands': RosPublisher('joint_commands', JointPositions, queue_size=10),
        # Publish joint commands to the robot to make it move
        'joint_publisher': RosSubscriber('joint_commands', JointPositions, tcp_server),
        # Subscribe to xyz_rpy_grip commands (these will come from AI or sliders) then rebroadcast joint_commands
        'xyz_rpy_g_command': RosPublisher('xyz_rpy_g_command', PositionCommand, queue_size=10),
        # Subscribe to xyz_quat_grip commands (these will come from the vr controller), and rebroadcast the rpy_grip equivalent
        'xyz_quat': RosPublisher('xyz_quat_g_command', QuaternionCommand, queue_size=10),
        # Subscribe to the various state elements, e.g cube pos, door pos, img eventually
        # TODO
        'state': RosPublisher('state', ArmState, queue_size=10)
    })
    
    rospy.spin()


if __name__ == "__main__":
    main()

