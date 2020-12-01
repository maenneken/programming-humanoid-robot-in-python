'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
import numpy as np

class AngleInterpolationAgent(PIDAgent):
    global start_time
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        self.start_time=0
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        if self.start_time == 0:
            self.start_time = perception.time

        i = 0
        current_time = perception.time-self.start_time
        for joint in keyframes[0]:
            k = 0
            k0 = 0
            for time in keyframes[1][i]:
                if time >= current_time: #get index of p0
                    k0 = k-1
                    break
                k += 1

            #calk B(current_time)
            t = (current_time - keyframes[1][i][k0]) / (keyframes[1][i][(k0 + 1)] - keyframes[1][i][k0])  # t=(s-s0)/(s1-s0)
            if k >= k0 >= 0 and 1 >= t >= 0: #check if we are at last elem or first
                p0 = keyframes[2][i][k0][0]
                p1 = keyframes[2][i][k0][2][1] + p0
                p3 = keyframes[2][i][k0 + 1][0]
                p2 = keyframes[2][i][k0+1][1][1] + p3


                print(t)
                target_joints[joint] = np.power(1-t, 3) * p0 + 3 * np.power(1-t, 2) * t * p1 + 3 * (1-t) * np.power(t, 2) * p2 + np.power(t, 3) * p3

            i += 1
        # YOUR CODE HERE


        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
