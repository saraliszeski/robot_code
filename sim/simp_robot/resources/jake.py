import pybullet as p
import numpy as np
import os
import pybullet_data

class Jake:
    def __init__(self, client):
        self.client = client
        f_name = os.path.join(os.path.dirname(__file__), 'jake.urdf')
        self.jake = p.loadURDF(f_name,
                              basePosition=[0, 0, 0.18],
                              physicsClientId=client)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        planeId = p.loadURDF("plane.urdf")
        robotStartPos = [0,0,0.18]
        robotStartOrientation = p.getQuaternionFromEuler([0,0,0])
        self.mode = p.POSITION_CONTROL
        # robotId = p.loadURDF("jake.urdf", robotStartPos, robotStartOrientation)
        self.joints = [2,4,6,10,12,14]

    def get_ids(self):
        return self.client, self.jake

    def apply_action(self, action):
        #get state of each joint then add the action'd degree to it
        for joint in range(len(self.joints)):
            angle_addition = action[joint]
            curr_angle = p.getJointState(self.jake, self.joints[joint])[0]
            p.setJointMotorControl2(self.jake, self.joints[joint], controlMode=self.mode, targetPosition=curr_angle + angle_addition)
        
    
    def get_observation(self):
        pos, orn = p.getBasePositionAndOrientation(self.jake, self.client)
        vel = p.getBaseVelocity(self.jake, self.client)[0][0]
        t = (pos + orn)
        t = t + (vel, )
        return t
