'''
Requirements for running this demo:
1. python2.7 ROS Kinetic mpi4py Stage PyTorch
2. roscore 
3. rosrun stage_ros_add_pose_and_crash stageros worlds/circle.world

Tips for using gpu:
In ppo.py file:
line @@96 - 98: .cpu() --> .cuda()

In demo.py
line @@41: .cpu() --> .cuda()
line @@42: delete ",map_location..." --> state_dict = torch.load(file)
'''
import os
import numpy as np
import rospy
import torch
import torch.nn as nn
from mpi4py import MPI

from torch.optim import Adam
from collections import deque

from model.net import MLPPolicy, CNNPolicy
from circle_world import StageWorld
from model.ppo import generate_action_no_sampling, transform_buffer


# Set parameters of env
LASER_HIST = 3
NUM_ENV = 10    # the number of agents in the environment
OBS_SIZE = 512  # number of leaserbeam
action_bound = [[0, -1], [1, 1]]    # the limitation of velocity

# Set env and agent policy
env = StageWorld(OBS_SIZE, index=0, num_env=NUM_ENV)    #index is useful for parallel programming, 0 is for the first agent
env.reset_world()
env.reset_pose()

trained_model_file = 'policy/stage2.pth'
policy = CNNPolicy(frames=LASER_HIST, action_space=2) 
policy.cpu()    # policy.cuda() for gpu
state_dict = torch.load(trained_model_file,map_location=torch.device('cpu'))    #torch.load(trained_model_file) for gpu
policy.load_state_dict(state_dict)

# Set fake obstacles by fake lidar info
obstacles_location = []
for i in range(NUM_ENV-1):
    obstacles_location.append(np.zeros(512))
    obstacles_location[i][10*i:10*i+10] = 0.1*i + 0.05 

# Configure agent as host
obs = env.get_laser_observation()
for i in range(NUM_ENV-1):
    obs = obs - obstacles_location[i]   #add fake obstacles to the lidar data 

obs_stack = deque([obs, obs, obs])  #three dimensions are for matching the input of torch.from_numpy(s_list) fuction


# ************************************ Input ************************************
# agent: goal, start position
input_goal = [-25.00, -0.00]    # [-25,0] is for the cordinate system in circle wolrd
input_start_position = [0.5, 0.0, np.pi]    # x, y, theta

env.goal_point = input_goal
goal = np.asarray(env.get_local_goal())     # transfer to robot based codinate system
speed = np.asarray(env.get_self_speed())
state = [obs_stack, goal, speed]

env.control_pose(input_start_position)


# ************************************ Output ************************************
# agent: postion(x,y,theta),velocity(v,angular)

_,scaled_action =generate_action_no_sampling(env=env, state_list=[state], policy=policy, action_bound=action_bound)
action = scaled_action[0]
print('velocity : ', action[0], 'velocity_angular : ', action[1])


env.control_vel(action)      #involke the ros system publisher to send velocity to ros system
rospy.sleep(0.01)
# the following output function is depending on callback function in ros.
print('positon: x, y, theta', env.get_self_state())
print('speed of agent: v, angular ', env.get_self_speed())