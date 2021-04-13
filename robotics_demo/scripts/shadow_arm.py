#!/usr/bin/env python
import pybullet as p
import numpy as np
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print("current_dir=" + currentdir)
from pybullet_utils import bullet_client
os.sys.path.insert(0, currentdir)
np.set_printoptions(suppress=True)
class InverseKinematicsSolver():
    '''
    A class to do multiple steps of IK using an arm at baseline, which means it converges onto one solution and is less finnicky with rpy
    While this isn't necessary for the panda, for some reason it is with the UR5
    '''
    
    def __init__(self, connection_mode=p.DIRECT, base_pos=[0,0,0], base_orn=[-np.pi/2,-np.pi/2,0], ee_index=8, default_joints=[0, -0.89202866, -0.22291248, -1.91023546, -1.00971437,  0.67797174,  0.0009883,0]):
        self.p = bullet_client.BulletClient(connection_mode=connection_mode)
        self.p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP ,1)
        filename= "/ur5v2/ur5_robot.urdf"
        self.ur5 = self.p.loadURDF(currentdir + filename,
                                                 base_pos,
                                                 p.getQuaternionFromEuler(base_orn), useFixedBase=True)
        self.n_joints = self.p.getNumJoints(self.ur5)

        # [base, shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint,wrist_3_joint]
        # base is a nothing but a world link appears required for unity
        self.joints_indices = [0,1,2,3,4,5, 6]
        self.jointNames = [self.p.getJointInfo(self.ur5, i)[1] for i in self.joints_indices]

        self.arm_joint_positions = None
        self.arm_joint_velocity = None

        self.default_joints = default_joints
        self.ee_index=ee_index
        self.set_states(self.default_joints)
        self.ll = np.array([-np.pi, (-np.pi/2)*0.9, -np.pi, -np.pi, -np.pi, -np.pi])
        self.ul = np.array([np.pi, np.pi, 0.0, np.pi, np.pi, np.pi])

    # sets the arm to the current joint positions so IK calcs are accurate
    def set_states(self,states):
        
        for idx, i in enumerate(self.joints_indices):
            # print_output("states {}".format(i))
            
            self.p.resetJointState(self.ur5, i, states[idx])

    def get_position(self):
        return self.p.getLinkState(self.ur5,self.ee_index)[0:2]

    def calc_angles(self, pos,ori, current_states=None):
        
        ori = p.getQuaternionFromEuler(ori)
        # if current_states is None:
        #     self.set_states(self.default_joints)
        # else:
        #     self.set_states(current_states)
        for i in range(0,3): # converge on the solution
            angles = [0]+list(np.clip(self.p.calculateInverseKinematics(self.ur5,self.ee_index, pos,ori), self.ll, self.ul))
            #print(np.array(angles)*180/np.pi)
            #
            
            self.set_states(angles)
        
        return np.clip(self.p.calculateInverseKinematics(self.ur5,self.ee_index, pos,ori), self.ll, self.ul)[0:7]


    def reset(self):
        self.set_states(self.default_joints)





if __name__ == "__main__":
    robot = InverseKinematicsSolver(connection_mode=p.GUI)
    import time
    
    def add_xyz_rpy_controls():
        controls = []
        controls.append(p.addUserDebugParameter("X", -1, 1, 0.4))
        controls.append(p.addUserDebugParameter("Y", -1, 1, 0.2))
        controls.append(p.addUserDebugParameter("Z", -1, 1, 0.0))
        controls.append(p.addUserDebugParameter("R", -4, 4, 0))
        controls.append(p.addUserDebugParameter("P", -4, 4, 0))
        controls.append(p.addUserDebugParameter("Y", -4,4, 0))
        controls.append(p.addUserDebugParameter("grip", 0, 1, 0))
        return controls

    def add_joint_controls():

        for i, obj in enumerate(robot.default_joints):
            p.addUserDebugParameter(str(i), -2*np.pi, 2*np.pi, obj)


    joint_control = False #Tru

    if joint_control:
        add_joint_controls()
    else:
        controls = add_xyz_rpy_controls()

    for i in range(1000000):
        if joint_control:
            poses  = []
            for i in range(0, len(robot.default_joints)):
                poses.append(p.readUserDebugParameter(i))
            robot.set_states(poses)
            info = robot.get_position()
            print(p.getEulerFromQuaternion(info[1]))
            print(info[1])
        else:
            action = []
            for i in range(0, len(controls)):
                action.append(p.readUserDebugParameter(i))
            poses = robot.calc_angles(action[0:3],action[3:6])
            print(np.array(poses))
            info = robot.get_position()
            #print(p.getEulerFromQuaternion(info[1]))
            #print(info[1])



