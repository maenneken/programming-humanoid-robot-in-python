'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np
import math
from scipy.linalg import pinv

class InverseKinematicsAgent(ForwardKinematicsAgent):
    '''gives last Matrix of effector so the offset is correct '''
    def getTe(self,effector_name):
        Te= identity(4)
        if('Arm' in effector_name):
            Te[3]=np.array([57.75,0,0,1])
        if('Leg' in effector_name):
            Te[3]=np.array([0,0,-45.19,1])
        return Te

    def from_trans(self,T):
        x=T[3,0]
        y=T[3,1]
        z=T[3,2]
        '''Pitch=-math.asin(T.T[2,0])
        Roll=math.atan2(T.T[2,1]/np.cos(Pitch),T.T[2,2]/np.cos(Pitch))
        Yaw=math.atan2(T.T[1,0],T.T[0,0])'''
        return x,y,z

    def getRotationAxes(self,joint_name):
        R=np.zeros(3)
        if('Roll' in joint_name):#x-Rotation
            R+=[1,0,0]
        if('Pitch' in joint_name):#y-Rotation
            R+=[0,1,0]
        if('Yaw' in joint_name): #z-Rotation
            R+=[0,0,1]
        return R

    def getJacobian(self,chain,dT):
        '''creates jacobian Matrix like https://medium.com/unity3danimation/overview-of-jacobian-ik-a33939639ab2 '''
        J = np.zeros((3,len(chain)))

        for i in range(len(chain)):
            R = self.getRotationAxes(chain[i])
            J[:,i] = np.cross(R,np.asarray(dT[:,i].T)[0])
        return J


    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        chain=self.chains.get(effector_name)
        theta = np.random.random(len(chain)) * 1e-5
        lambda_ = 1
        max_step = 0.1
        joint_angles = []
        joints={}
        Toffset=self.getTe(effector_name)

        for i in range(1000):
            for i in range(len(chain)):
                joints[chain[i]]=theta[i]
            self.forward_kinematics(joints)
            Te = np.matrix([self.from_trans(np.matmul(self.transforms[chain[-1]],Toffset))]).T
            e = np.matrix([self.from_trans(transform)]).T - Te
            e[e > max_step] = max_step
            e[e < -max_step] = -max_step
            T = np.matrix([self.from_trans(self.transforms[j]) for j in chain]).T
            dT = Te - T
            J = self.getJacobian(chain,dT)
            d_theta = lambda_ * pinv(J) * e
            theta += np.asarray(d_theta.T)[0]
            if  np.linalg.norm(d_theta) < 1e-4:
                break

        joint_angles=theta
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        joint_angles=self.inverse_kinematics(effector_name, transform)
        chain=self.chains.get(effector_name)
        self.keyframes = ([], [], [])  # the result joint angles have to fill in
        names = list()
        times = list()
        keys = list()
        for i in range(len(chain)):
            names.append(chain[i])
            times.append([1.0,2.0])
            keys.append([[joint_angles[i]/2,[0,0,0],[0,0,0]],[joint_angles[i]/2,[0,0,0],[0,0,0]]])
        self.keyframes = (names,times, keys)  # the result joint angles have to fill in
        self.start_time=0
if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
