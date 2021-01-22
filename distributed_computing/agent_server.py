'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
import threading
from numpy.matlib import identity
import numpy as np
import time
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent
from forward_kinematics import ForwardKinematicsAgent
from recognize_posture import PostureRecognitionAgent
from angle_interpolation import AngleInterpolationAgent

class ServerAgent(InverseKinematicsAgent,ForwardKinematicsAgent,PostureRecognitionAgent,AngleInterpolationAgent):
    '''ServerAgent provides RPC service'''
    # YOUR CODE HERE

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.perception.joint[joint_name]

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller'''
        # YOUR CODE HERE
        self.target_joints[joint_name]=angle
        return 'done'


    def get_posture(self):
        '''return current posture of robot'''
        return self.posture


    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed'''
        self.keyframes=keyframes
        self.start_time=0
        times=keyframes[1]
        t=0
        for i in range(len(times)):
            tmp=max(times[i])
            if(tmp>t):
                t=tmp
        time.sleep(t)
        return 'done'

    def get_transform(self, name):
        '''get transform with given name'''
        return self.transforms[name].tolist()


    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        self.set_transforms(effector_name, np.asmatrix(transform))
        return 'done'



if __name__ == '__main__':
    agent = ServerAgent()
    server = SimpleXMLRPCServer(('127.0.0.1', 9000), logRequests=True, allow_none=True)
    server.register_function(pow)
    server.register_instance(agent)
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.start()
    agent.run()
