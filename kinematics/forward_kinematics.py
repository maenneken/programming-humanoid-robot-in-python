'''In this exercise you need to implement forward kinematics for NAO robot

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
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw','LElbowRoll','LWristYaw', 'LHand'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch','RAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch','LAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw','RElbowRoll','RWristYaw', 'RHand'],


                       }

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
        s=sin(joint_angle)
        c=cos(joint_angle)
        if('Roll' in joint_name):#x-Rotation
            T[1]=np.array([0,c,-s,0])
            T[2]=np.array([0,s,c,0])
        if('Pitch' in joint_name):#y-Rotation
            T[0]=np.array([c,0,s,0])
            T[2]=np.array([-s,0,c,0])
        if('Yaw' in joint_name): #z-Rotation
            T[0]=np.array([c,s,0,0])
            T[1]=np.array([-s,c,0,0])

        #offsets
        #head
        if(joint_name=='HeadYaw'):
            T[3]=np.array(0,0,126.5,1)
        #Arms
        if(joint_name=='LShoulderPitch'):
            T[3]=np.array(0,98,100,1)
        if(joint_name=='RShoulderPitch'):
            T[3]=np.array(0,-98,100,1)
        if('ElbowYaw'in joint_name):
            T[3]=np.array(105,15,0,1)
        if('WrisYaw'in joint_name):
            T[3]=np.array(55.95,0,0,1)
        #Legs
        if(joint_name=='LHipYawPitch'):
            T[3]=np.array(0,50,-85,1)
        if(joint_name=='RHipYawPitch'):
            T[3]=np.array(0,-50,-85,1)
        if('KneePitch'in joint_name):
            T[3]=np.array(0,0,-100,1)
        if('AnklePitch'in joint_name):
            T[3]=np.array(0,0,-102.9,1)

        
        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
