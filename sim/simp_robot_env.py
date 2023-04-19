import gym
import numpy as np
import pybullet as p
import pybullet_data
import math
import matplotlib.pyplot as plt
from simp_robot.resources.goal import Goal
from simp_robot.resources.jake import Jake

import gym
from gym import spaces
import pybullet as p
import pybullet_data
import numpy as np
import simp_robot


class SimpRobotEnv(gym.Env):
    metadata = {'render.modes': ['rgb_array']}  
  
    def __init__(self):
        self.action_space = gym.spaces.box.Box(
            low=np.array([-1,-1,-1,-1,-1,-1]),
            high=np.array([1,1,1,1,1,1]))
        self.observation_space = gym.spaces.box.Box(
            #x,y,z,roll,pitch,yaw, reward??
            low=np.array([-np.inf, -np.inf,-np.inf,-np.inf,-np.inf,-np.inf, -np.inf,-np.inf,-np.inf,-np.inf]),
            high=np.array([np.inf, np.inf,np.inf,np.inf,np.inf,np.inf, np.inf,np.inf,np.inf,np.inf]))
        self.np_random, _ = gym.utils.seeding.np_random()
        self.client = p.connect(p.DIRECT)
        # self.startpos =  p.getBasePositionAndOrientation(self.jake.get_ids()[0])
        p.setTimeStep(1/30, self.client)
        self.jake = None
        self.goal = None
        self.done = False
        self.prev_dist_to_goal = None
        self.render = False
        self.rendered_img = None
        self.render_rot_matrix = None
        self.reset()
        
    def step(self, action):
        old_pos, old_orn = p.getBasePositionAndOrientation(self.jake.get_ids()[0])
        old_x = old_pos[0]
        old_y = old_pos[1]
        self.jake.apply_action(action)
        p.stepSimulation()
        rob_ob = self.jake.get_observation()
        
        
        dist_to_goal = math.sqrt(((rob_ob[0] - self.goal[0]) ** 2 +
                                  (rob_ob[1] - self.goal[1]) ** 2))
        #compute reward
        robotPos, robotOrn = p.getBasePositionAndOrientation(self.jake.get_ids()[0])
         # extract the robot's current position and orientation
        # pos, orn = state[:3], state[3:7]

        # calculate the distance to the target position (10,0,0)
        target_pos = np.array([10, 0, 0])
        distance = np.linalg.norm(robotPos - target_pos)

        # calculate the deviation from standing upright
        z_axis = np.array([0, 0, 1])
        deviation = np.abs(np.dot(robotOrn[0:3], z_axis))

        # calculate a reward based on the distance to the target and the deviation from upright
        target_reward = 1 - 0.01 * distance
        upright_reward = 1 - 0.05 * deviation
        self.reward += target_reward * upright_reward

        # v_x = p.getBaseVelocity(self.jake.get_ids()[0], self.client)[0][0]
        # y = np.abs(robotPos[1])
        # z = np.abs((self.start_pos[2]-robotPos[2]))
        # u = 0
        # new_x = robotPos[0]
        # healthy_reward = 1
        # forward_reward = 5*(new_x - old_x)/(5/30)
        # self.reward +=  healthy_reward + forward_reward - 10*robotPos[1]-self.reward
        # self.reward = max(self.reward, math.sqrt((robotPos[0]**2 + robotPos[1]**2 ))-(self.goal[0]-robotPos[0])**2)
        #done if fallen over
        if deviation > 0.1:
            print('Done due to fall')
            self.reward -= 100
            self.done = True
        
        #done if reaches goal
        elif dist_to_goal < 0.1:
            print("Done due to goal proximity")
            self.done = True
            reward = 50
       
        ob = np.array(rob_ob+self.goal, dtype=np.float32)
        return ob, self.reward, self.done, dict()

    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0,0,-10)
        self.jake = Jake(self.client)
        self.jake.basePosition= [0, 0, 0.18]
        self.goal = (10, 0)
        jake_ob = self.jake.get_observation()
        self.start_pos = jake_ob[0:3]
        
        self.reward = 0
        self.done = False
        Goal(self.client, self.goal)
            # planeId = p.loadURDF("plane.urdf")
        return np.array(jake_ob+self.goal)

    def render(self, mode='human'):
        if self.rendered_img is None:
            self.rendered_img = plt.imshow(np.zeros((100, 100, 4)))

        # Base information
        car_id, client_id = self.jake.get_ids()
        proj_matrix = p.computeProjectionMatrixFOV(fov=800, aspect=1,
                                                   nearVal=0.01, farVal=100)
        pos, ori = [list(l) for l in p.getBasePositionAndOrientation(car_id, client_id)]
        pos[2] = 0.2

        # Rotate camera direction
        rot_mat = np.array(p.getMatrixFromQuaternion(ori)).reshape(3, 3)
        camera_vec = np.matmul(rot_mat, [1, 0, 0])
        up_vec = np.matmul(rot_mat, np.array([0, 0, 1]))
        view_matrix = p.computeViewMatrix(pos, pos + camera_vec, up_vec)

        # Display image
        frame = p.getCameraImage(100, 100, view_matrix, proj_matrix)[2]
        frame = np.reshape(frame, (100, 100, 4))
        self.rendered_img.set_data(frame)
        plt.draw()
        plt.pause(.00001)

    def close(self):
        p.disconnect(self.client)
        
    def seed(self, seed=None): 
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]