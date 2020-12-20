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
        if('RHipYawPitch'==joint_name):
            R=[0,1,-1]
        return R

    def getJacobian(self,chain,dT):
        '''creates jacobian Matrix like https://medium.com/unity3danimation/overview-of-jacobian-ik-a33939639ab2 '''
        J = np.zeros((3,len(chain)))

        for i in range(len(chain)):
            R = self.getRotationAxes(chain[i])
            J[:,i] = np.cross(R,np.asarray(dT[:,i].T)[0])
        return J

    def possibleAngle(self,theta,chain):
        i=0;
        for joint_name in chain:
                #Head
                if(joint_name=='HeadYaw'):
                    if(theta[i]<-2.0857):
                        theta[i]=-2.0857
                    if(theta[i]>2.0857):
                        theta[i]=2.0857
                if(joint_name=='HeadPitch'):
                    if(theta[i]<-0.6720):
                        theta[i]=-0.6720
                    if(theta[i]>0.5149):
                        theta[i]=0.5149
                #Arms
                if('ShoulderPitch' in joint_name):
                    if(theta[i]<-2.0857):
                        theta[i]=-2.0857
                    if(theta[i]>2.0857):
                        theta[i]=2.0857
                if('RShoulderRoll'==joint_name):
                    if(theta[i]<-0.3142):
                        theta[i]=-0.3142
                    if(theta[i]>1.3265):
                        theta[i]=1.3265
                if('LShoulderRoll'==joint_name):
                    if(theta[i]<-1.3265):
                        theta[i]=-1.3265
                    if(theta[i]>0.3142):
                        theta[i]=0.3142
                if('ElbowYaw'in joint_name):
                    if(theta[i]<-2.0857):
                        theta[i]=-2.0857
                    if(theta[i]>2.0857):
                        theta[i]=2.0857
                if('RElbowRoll'==joint_name):
                    if(theta[i]>6.2832):
                        theta[i]=0.0349
                    if(0.0349>theta[i]>1.5446):
                        theta[i]=1.5446
                if('LElbowRoll'==joint_name):
                    if(theta[i]<0.0349):
                        theta[i]=0.0349
                    if(theta[i]>1.5446):
                        theta[i]=1.5446
                #Legs
                if('HipYawPitch'in joint_name):
                    if(theta[i]<-1.45303):
                        theta[i]=-1.45303
                    if(theta[i]>0.740810):
                        theta[i]=0.740810
                if('LHipRoll'==joint_name):
                    if(theta[i]<-0.379472):
                        theta[i]=-0.379472
                    if(theta[i]>0.790477):
                        theta[i]=0.790477
                if('RHipRoll'==joint_name):
                    if(theta[i]<-0.790477):
                        theta[i]=-0.790477
                    if(theta[i]>0.379472):
                        theta[i]=0.379472
                if('HipPitch'in joint_name):
                    if(theta[i]<-1.535889):
                        theta[i]=-1.535889
                    if(theta[i]>0.484090):
                        theta[i]=0.484090
                if('LKneePitch'== joint_name):
                    if(theta[i]<-0.092346):
                        theta[i]=-0.092346
                    if(theta[i]>2.112528):
                        theta[i]=2.112528
                if('RKneePitch'== joint_name):
                    if(theta[i]<-0.103083):
                        theta[i]=-0.103083
                    if(theta[i]>2.120198):
                        theta[i]=2.120198
                if('AnklePitch'in joint_name):
                    if(theta[i]<-1.89516):
                        theta[i]=-1.89516
                    if(theta[i]>0.922747):
                        theta[i]=0.922747
                if('LAnkleRoll'==joint_name):
                    if(theta[i]<-0.397880):
                        theta[i]=-0.397880
                    if(theta[i]>0.769001):
                        theta[i]=0.769001
                if('RAnkleRoll'==joint_name):
                    if(theta[i]<-0.768992):
                        theta[i]=-0.768992
                    if(theta[i]>0.397935):
                        theta[i]=0.397935
                i+=1

        return theta

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        chain=self.chains.get(effector_name)
        theta = np.zeros(len(chain))
        lambda_ = 1
        max_step = 0.1
        joint_angles = []
        joints={}
        Toffset=self.getTe(effector_name)

        for i in range(10000):
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
            theta=self.possibleAngle(theta,chain)
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
    T = identity(4) #coordinates are global
    T[-1,0]= 0 # x in mm
    T[-1, 1] = 120 #y in mm
    T[-1, 2] = 0#z in mm
    agent.set_transforms('LArm', T)
    agent.run()
