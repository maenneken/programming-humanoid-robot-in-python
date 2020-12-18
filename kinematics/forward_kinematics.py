'''In this exercise you need to implement forward kinematics for sNAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys
import numpy as np
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity

from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw','LElbowRoll'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch','RAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch','LAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw','RElbowRoll']}




    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''

        T = identity(4)
        M = identity(4)
        s=np.sin(joint_angle)
        c=np.cos(joint_angle)
        if('Roll' in joint_name):#x-Rotation
            M[1]=np.array([0,c,-s,0])
            M[2]=np.array([0,s,c,0])
            T=np.matmul(T,M)
        if('Pitch' in joint_name):#y-Rotation
            M[0]=np.array([c,0,s,0])
            M[2]=np.array([-s,0,c,0])
            T=np.matmul(T,M)
        if('Yaw' in joint_name): #z-Rotation
            M[0]=np.array([c,s,0,0])
            M[1]=np.array([-s,c,0,0])
            T=np.matmul(T,M)
        T=T.T
        #offsets
        #head
        if(joint_name=='HeadYaw'):
            T[3]=np.array([0,0,126.5,1])
        #Arms
        if(joint_name=='LShoulderPitch'):
            T[3]=np.array([0,98,100,1])
        if(joint_name=='RShoulderPitch'):
            T[3]=np.array([0,-98,100,1])
        if('ElbowYaw'in joint_name):
            T[3]=np.array([105,15,0,1])
        if('WrisYaw'in joint_name):
            T[3]=np.array([55.95,0,0,1])
        #Legs
        if(joint_name=='LHipYawPitch'):
            T[3]=np.array([0,50,-85,1])
        if(joint_name=='RHipYawPitch'):
            T[3]=np.array([0,-50,-85,19])
        if('KneePitch'in joint_name):
            T[3]=np.array([0,0,-100,1])
        if('AnklePitch'in joint_name):
            T[3]=np.array([0,0,-102.9,1])

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                if(joints.get(joint)!=None):
                    angle = joints[joint]
                    Tl = self.local_trans(joint, angle)
                    # YOUR CODE HERE
                    T=np.matmul(T,Tl)
                    self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
