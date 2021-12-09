import configparser
import torch
import gym

from crowd_nav.policy.cadrl import CADRL
from crowd_nav.policy.lstm_rl import LstmRL
from crowd_nav.policy.sarl import SARL

from crowd_sim.envs.utils.robot import Robot
# from crowd_sim.envs.utils.human import Human

# gpu = False
# device = torch.device("cuda:0" if torch.cuda.is_available() and gpu else "cpu")

device = 'cpu'
phase = 'test'
test_case = 1
policy_name = "lstm"
select_policy = {"cadrl":CADRL(),"lstm":LstmRL(),"sarl":SARL()}
# the path of training result which contains configs and rl mode
env_config_file = 'crowd_nav/data/output/env.config'             #path beginging without slash
policy_config_file = 'crowd_nav/data/output/policy.config'
model_weights = 'crowd_nav/data/output/rl_model_'+policy_name+'.pth'
print(model_weights)
# select policy
policy = select_policy[policy_name]     #{SARL(),CADRL(),LstmRL()}
policy_config = configparser.RawConfigParser()
policy_config.read(policy_config_file)
policy.configure(policy_config)
policy.get_model().load_state_dict(torch.load(model_weights))
policy.set_device(device)
policy.set_phase(phase)

# configure environment / obstacles
env_config = configparser.RawConfigParser()
env_config.read(env_config_file)
env = gym.make('CrowdSim-v0')   #env is inherited from CrowdSim class in crowd_sim.py
env.configure(env_config)

# configure robot
robot = Robot(env_config, 'robot')
robot.set_policy(policy)

env.set_robot(robot)    #pass robot parameters into env
ob = env.reset(phase,test_case)     #intial some parameters from .config file such as time_step,success_reward for other instances
policy.set_env(env)     #pass the env info into policy

# ************************************ Input ************************************
# robot: position, goal, radius, x_velocity, y_velocity, theta, radius
# position
robot_x = 0.1
robot_y = 0.1
# goal
goal_x = 1
goal_y = 1
# velocity
robot_vx = 1
robot_vy = 2
# oriantation
theta = 1
robot_radius = 0.3

# set robot info
robot.set(robot_x, robot_y, goal_x, goal_y, robot_vx, robot_vy, theta, robot_radius)

# obstacle: position, velocity, radius
# position
obstacle_x = [0.1,0.2,0.3,0.4,0.5]
obstacle_y = [0.1,0.2,0.3,0.4,0.5]
# velocity
obstacle_vx = [0.1,0.2,0.3,0.4,0.5]
obstacle_vy = [0.1,0.2,0.3,0.4,0.5]
obstacle_radius = 0.3

# initial obstacle instances and set value
for i in range(env_config.getint('sim','human_num')):
    env.humans[i].set(obstacle_x[i], obstacle_y[i], goal_x,goal_y, obstacle_vx[i], obstacle_vy[i], theta, obstacle_radius)
    ob[i]= env.humans[i].get_observable_state()

# ************************************ Output ************************************
# get action info
action = robot.act(ob)
position = robot.get_observable_state()
print('robot position (X,Y):', position.px, ",", position.py)
print('robot velocity (X,Y):', action.vx, ",", action.vy)
