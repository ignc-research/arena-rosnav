#! /usr/bin/env python
from operator import is_
from random import randint
import gym
from gym import spaces
from gym.spaces import space
from typing import Union
from stable_baselines3.common.env_checker import check_env
import yaml
from rl_agent.utils.observation_collector import ObservationCollector
from rl_agent.utils.reward import RewardCalculator
from rl_agent.utils.debug import timeit
from task_generator.tasks import ABSTask
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from flatland_msgs.srv import StepWorld, StepWorldRequest
import time


class FlatlandEnv(gym.Env):
    """Custom Environment that follows gym interface"""

    def __init__(self, task: ABSTask, robot_yaml_path: str, settings_yaml_path: str, is_action_space_discrete, safe_dist: float = None, goal_radius: float = 0.1, max_steps_per_episode=100):
        """Default env
        Flatland yaml node check the entries in the yaml file, therefore other robot related parameters cound only be saved in an other file.
        TODO : write an uniform yaml paser node to handel with multiple yaml files.



        Args:
            task (ABSTask): [description]
            robot_yaml_path (str): [description]
            setting_yaml_path ([type]): [description]
            is_action_space_discrete (bool): [description]
            safe_dist (float, optional): [description]. Defaults to None.
            goal_radius (float, optional): [description]. Defaults to 0.1.
        """
        super(FlatlandEnv, self).__init__()
        # Define action and observation space
        # They must be gym.spaces objects

        self._is_action_space_discrete = is_action_space_discrete
        self.setup_by_configuration(robot_yaml_path, settings_yaml_path)
        # observation collector
        self.observation_collector = ObservationCollector(
            self._laser_num_beams, self._laser_max_range)
        self.observation_space = self.observation_collector.get_observation_space()

        # reward calculator
        if safe_dist is None:
            safe_dist = 1.1*self._robot_radius

        self.reward_calculator = RewardCalculator(
            safe_dist=1.1*self._robot_radius, goal_radius=goal_radius)

        # action agent publisher
        self.agent_action_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # service clients
        self._service_name_step = '/step_world'
        self._sim_step_client = rospy.ServiceProxy(
            self._service_name_step, StepWorld)
        self.task = task
        self._steps_curr_episode = 0
        self._max_steps_per_episode = max_steps_per_episode
        # # get observation
        # obs=self.observation_collector.get_observations()

    def setup_by_configuration(self, robot_yaml_path: str, settings_yaml_path: str):
        """get the configuration from the yaml file, including robot radius, discrete action space and continuous action space.

        Args:
            robot_yaml_path (str): [description]
        """
        with open(robot_yaml_path, 'r') as fd:
            robot_data = yaml.safe_load(fd)
            # get robot radius
            for body in robot_data['bodies']:
                if body['name'] == "base_footprint":
                    for footprint in body['footprints']:
                        if footprint['type'] == 'circle':
                            self._robot_radius = footprint.setdefault(
                                'radius', 0.2)
            # get laser related information
            for plugin in robot_data['plugins']:
                if plugin['type'] == 'Laser':
                    laser_angle_min = plugin['angle']['min']
                    laser_angle_max = plugin['angle']['max']
                    laser_angle_increment = plugin['angle']['increment']
                    self._laser_num_beams = int(
                        round((laser_angle_max-laser_angle_min)/laser_angle_increment)+1)
                    self._laser_max_range = plugin['range']

        with open(settings_yaml_path, 'r') as fd:
            setting_data = yaml.safe_load(fd)
            if self._is_action_space_discrete:
                # self._discrete_actions is a list, each element is a dict with the keys ["name", 'linear','angular']
                self._discrete_acitons = setting_data['robot']['discrete_actions']
                self.action_space = spaces.Discrete(
                    len(self._discrete_acitons))
            else:
                linear_range = setting_data['robot']['continuous_actions']['linear_range']
                angular_range = setting_data['robot']['continuous_actions']['angular_range']
                self.action_space = spaces.Box(low=np.array([linear_range[0], angular_range[0]]),
                                               high=np.array(
                                                   [linear_range[1], angular_range[1]]),
                                               dtype=np.float)

    def _pub_action(self, action):
        action_msg = Twist()
        if self._is_action_space_discrete:
            action_msg.linear.x = self._discrete_acitons[action]['linear']
            action_msg.angular.z = self._discrete_acitons[action]['angular']
        else:
            action_msg.linear.x = action[0]
            action_msg.angular.z = action[1]
        self.agent_action_pub.publish(action_msg)

    def step(self, action):
        self._pub_action(action)
        self._steps_curr_episode += 1
        # wait for new observations
        s = time.time()
        merged_obs, obs_dict = self.observation_collector.get_observations()
        print("get observation: {}".format(time.time()-s))

        # calculate reward
        reward, reward_info = self.reward_calculator.get_reward(
            obs_dict['laser_scan'], obs_dict['goal_in_robot_frame'])
        done = reward_info['is_done']
        # info
        if not done:
            done = self._steps_curr_episode > self._max_steps_per_episode
        info = {}
        return merged_obs, reward, done, info

    def reset(self):

        # set task
        # regenerate start position end goal position of the robot and change the obstacles accordingly
        self.agent_action_pub.publish(Twist())
        self._sim_step_client()
        self.task.reset()
        self.reward_calculator.reset()
        self._steps_curr_episode = 0
        obs, _ = self.observation_collector.get_observations()
        return obs  # reward, done, info can't be included

    def close(self):
        pass


if __name__ == '__main__':

    rospy.init_node('flatland_gym_env', anonymous=True)
    print("start")

    flatland_env = FlatlandEnv()
    check_env(flatland_env, warn=True)

    # init env
    obs = flatland_env.reset()

    # run model
    n_steps = 200
    for step in range(n_steps):
        # action, _states = model.predict(obs)
        action = flatland_env.action_space.sample()

        obs, rewards, done, info = flatland_env.step(action)

        time.sleep(0.1)
