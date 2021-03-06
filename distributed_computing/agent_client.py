'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import threading
import requests
import json
import time
import xmlrpc.client
from keyframes import hello
from numpy.matlib import identity
class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)
    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        server = xmlrpc.client.ServerProxy('http://localhost:9000')
        thread = threading.Thread(target = server.execute_keyframes_parallel, args=(keyframes))
        thread.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        server = xmlrpc.client.ServerProxy('http://localhost:9000')
        thread = threading.Thread(target = server.set_transform,args=(effector_name,transform))
        thread.start()

class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''

    def __init__(self):
        self.server = xmlrpc.client.ServerProxy('http://localhost:9000')
        self.post = PostHandler(self)

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.server.get_angle(joint_name)


    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        return self.server.set_angle(joint_name,angle)

    def get_posture(self):
        '''return current posture of robot'''
        return self.server.get_posture()

    def execute_keyframes_parallel(self, names,times,keys):
        '''had to add this function because threading dosnt like keyframes as args
        '''

        return self.server.execute_keyframes_parallel(names,times,keys)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        self.server.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        return self.server.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        self.server.set_transform(effector_name, transform)

if __name__ == '__main__':
    agent = ClientAgent()
    agent.post.execute_keyframes(hello())
    print(agent.get_posture())
    print(agent.set_angle('HeadYaw',-1))
    print(agent.get_angle('HeadYaw'))
    print(agent.get_transform('HeadYaw'))
    agent.post.execute_keyframes(hello())
    print("hello send")
    T = [[1,0,0,0],
        [0,1,0,0],
        [0,0,1,0],
        [0,0,0,1]] #coordinates are global
    T[-1][0]= 0 # x in mm
    T[-1][1] = 120 #y in mm
    T[-1][2] = 0#z in mm
    agent.post.set_transform('LArm', T)
    print("all send")
