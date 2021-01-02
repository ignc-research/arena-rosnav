#! /usr/bin/env python
from random import randint
import gym
from gym import spaces
from stable_baselines3.common.env_checker import check_env

from rl_agent.utils.observation_collector import ObservationCollector
from rl_agent.utils.action_collector import ActionCollector
from rl_agent.utils.reward_collector import RewardCollector
from rl_agent.utils.debug import timeit
from task_generator.tasks import ABSTask

import rospy
from geometry_msgs.msg import Twist
from flatland_msgs.srv import StepWorld, StepWorldRequest
import time


class FlatlandEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self, task: ABSTask):
        super(FlatlandEnv, self).__init__()
        # Define action and observation space
        # They must be gym.spaces objects

        # observation collector
        self.observation_collector = ObservationCollector()
        self.observation_space = self.observation_collector.get_observation_space()

        # action collector
        self.action_collector = ActionCollector()
        self.action_space = self.action_collector.get_action_space()

        # reward collector
        self.reward_collector = RewardCollector()

        # action agent publisher
        #self.agent_action_pub = rospy.Publisher('drl_action_agent', Twist, queue_size=1)
        self.agent_action_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # service clients
        self._service_name_step = '/step_world'
        self._sim_step_client = rospy.ServiceProxy(
            self._service_name_step, StepWorld)
        self.task = task
        # # get observation
        # obs=self.observation_collector.get_observations()

    def step(self, action):
        s = time.time()
        # encode action to cmd_vel
        cmd_vel = self.action_collector.get_cmd_vel(action_id=action)
        print("cmd vel: {}".format(time.time()-s))
        # publish cmd_vel
        
        self.agent_action_pub.publish(cmd_vel)
        # wait for new observations
        s = time.time()
        obs = self.observation_collector.get_observations()
        print("get observation: {}".format(time.time()-s)) 

        # check if done
        done = self.is_done(obs)

        # calculate reward
        reward = self.reward_collector.get_reward(action)

        # info
        info = {}
        return obs, reward, done, info

    def reset(self):

        # set task
        # regenerate start position end goal position of the robot and change the obstacles accordingly
        s = time.time()
        self.agent_action_pub.publish(Twist())
        self._sim_step_client()
        self.task.reset()
        # for _ in range(20):
        #   self._sim_step_client()
        # print("reset needed time: {}".format(time.time() - s))
        # set goal, start global plan
        # get observation
        obs = self.observation_collector.get_observations()
        return obs  # reward, done, info can't be included

    def render(self, mode='human'):
        pass

    def close(self):
        pass

    def is_done(self, obs):
        # done=False
        # scan=obs[0]
        # robot_pose=obs[1]
        # subgol_pose=obs[2]
        # if(robot_pose[0]>10):
        #   done=True
        i = randint(0, 100)
        done = i > 95
        return done


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
