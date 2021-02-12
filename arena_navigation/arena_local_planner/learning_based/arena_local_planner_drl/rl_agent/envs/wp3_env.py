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
from geometry_msgs.msg import Twist, PoseStamped, Pose2D
from flatland_msgs.srv import StepWorld, StepWorldRequest
from nav_msgs.msg import Odometry, Path
import time
import math 
# for transformations
from tf.transformations import *

from arena_plan_msgs.msg import RobotState, RobotStateStamped

class wp3Env(gym.Env):
    """Custom Environment that follows gym interface"""

    def __init__(self, task: ABSTask, robot_yaml_path: str, settings_yaml_path: str, is_action_space_discrete, safe_dist: float = None, goal_radius: float = 0.1, max_steps_per_episode=100):
        """Default env
        Flatland yaml node check the entries in the yaml file, therefore other robot related parameters cound only be saved in an other file.
        TODO : write an uniform yaml paser node to handel with multiple yaml files.



        Args:
            task (ABSTask): [description]
            robot_yaml_path (str): [description]
            setting_yaml_path ([type]): [description]
            reward_fnc (str): [description]
            is_action_space_discrete (bool): [description]
            safe_dist (float, optional): [description]. Defaults to None.
            goal_radius (float, optional): [description]. Defaults to 0.1.
        """
        super(wp3Env, self).__init__()
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
            safe_dist = 1.5*self._robot_radius

        self.reward_calculator = RewardCalculator(
             safe_dist=1.1*self._robot_radius, goal_radius=goal_radius)

        #subscriber to infer callback and out of sleep loop
        #sub robot position and sub global goal 
        self._robot_state_sub = rospy.Subscriber('/odom', Odometry, self.cbRobotPosition)

        self._wp4train_sub = rospy.Subscriber('plan_manager/wp4train', PoseStamped, self.cbwp4train)
        self._globalPlan_sub = rospy.Subscriber('plan_manager/globalPlan', Path, self.cbglobalPlan)
        self._wp4train_reached =False
        # action agent publisher
        self.agent_action_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

        # service clients
        self._is_train_mode = rospy.get_param("train_mode")
        if self._is_train_mode:
            self._service_name_step = '/step_world'
            self._sim_step_client = rospy.ServiceProxy(
            self._service_name_step, StepWorld)
        self.task = task
        self.range_circle = 1.5
        self._steps_curr_episode = 0
        self._max_steps_per_episode = max_steps_per_episode

        #global variables for subscriber callbacks
        self._robot_pose = Pose2D()
        self._globalPlan = Path()
        self._wp4train = PoseStamped()

        self._previous_time = 0
        self._step_counter = 0
        # # get observation
        # obs=self.observation_collector.get_observations()

    def cbRobotPosition(self,msg):
        self._robot_pose.x = msg.pose.pose.position.x
        self._robot_pose.y = msg.pose.pose.position.y

    def cbglobalPlan(self,msg):
        self._globalPlan = msg

    def cbwp4train(self,msg):
        self._wp4train = msg
       
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
                                'radius', 0.3)*1.04
                        if footprint['radius']:
                            self._robot_radius = footprint['radius']*1.04
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
                angular_range = setting_data['robot']['continuous_actions']['angular_range']
                self.action_space = spaces.Box(low=np.array([angular_range[0]]),
                                               high=np.array([angular_range[1]]), dtype=np.float)

    def _pub_action(self, action):

        #todo instead of counter, wait for robot pose = wp4train pose
        if self._step_counter - self._previous_time > 80:
            self._previous_time = self._step_counter
            # _, obs_dict = self.observation_collector.get_observations()
            # robot_position = obs_dict['robot_position']
            action_msg = PoseStamped()
            #todo consider the distance to global path when choosing next optimal waypoint
            #caluclate range with current robot position and transform into posestamped message 
            # robot_position+(angle*range)
            #send a goal message as action, remeber to normalize the quaternions (put orientationw as 1) and set the frame id of the goal! 
            angle_grad = math.degrees(action[0]) # e.g. 90 degrees
            action_msg.pose.position.x = self._wp4train.pose.position.x + (self.range_circle*math.cos(angle_grad))         
            action_msg.pose.position.y = self._wp4train.pose.position.y + (self.range_circle*math.sin(angle_grad))   
            dist_robot_goal = np.array([(self.range_circle*math.cos(angle_grad)) ,(self.range_circle*math.sin(angle_grad))])
            dist_norm = np.linalg.norm(dist_robot_goal)
            action_msg.pose.orientation.w = 1
            action_msg.header.frame_id ="map"
            self.agent_action_pub.publish(action_msg)
            print(angle_grad)
            #rospy.sleep(1)
            #print("chosen action:  {0}, deegrees:   {1}, sum: {2}, cos(): {3}, robot_position:   {4}".format(action[0], math.degrees(action[0]), (self.range_circle*np.cos(math.degrees(action[0]))),np.cos(math.degrees(action[0])), robot_position ))

            # while (math.isclose(self._robot_pose.x, action_msg.pose.position.x, rel_tol=0.2) and math.isclose(self._robot_pose.y, action_msg.pose.position.y, rel_tol=0.2)) == False :
            #     #log laserscan and write into history buffer
            #     print("positions not the same")
            #     print(self._robot_pose.x)
            #     print(action_msg.pose.position.x)
            #     print(self._robot_pose.y)
            #     print(action_msg.pose.position.y)
            #     print((math.isclose(self._robot_pose.x, action_msg.pose.position.x, rel_tol=0.2) and math.isclose(self._robot_pose.y, action_msg.pose.position.y, rel_tol=0.5)))
            #     #time.sleep(1)
    def step(self, action):
        
        """
        done_reasons:   0   -   exceeded max steps
                        1   -   collision with obstacle
                        2   -   goal reached
        """
        self._step_counter += 1
        # wait for about 10 steps until do next step, todo wait until agent reaches the goal (robot pose = goal pose from move base simple goal)
        
        self._pub_action(action)
        ##todo: wait for robot_pos = goal pos

        
        # wait for new observations
        s = time.time()
        merged_obs, obs_dict = self.observation_collector.get_observations()
        # print("get observation: {}".format(time.time()-s))

        # calculate reward
        reward, reward_info = self.reward_calculator.get_reward(
            obs_dict['laser_scan'], obs_dict['goal_in_robot_frame'])
        done = reward_info['is_done']

        print("reward:  {}".format(reward))
        
        # info
        info = {}
        if done:
            info['done_reason'] = reward_info['done_reason']
        else:
            if self._steps_curr_episode == self._max_steps_per_episode:
                done = True
                info['done_reason'] = 0

        return merged_obs, reward, done, info

    def reset(self):
        
        self._previous_time = -100 # some negative number to infer first run
        self._step_counter = 0
        # set task
        # regenerate start position end goal position of the robot and change the obstacles accordingly
        self.agent_action_pub.publish(PoseStamped())
        if self._is_train_mode:
            self._sim_step_client()
        self.task.reset()
        self.reward_calculator.reset()
        self._steps_curr_episode = 0
        obs, _ = self.observation_collector.get_observations()
        return obs  # reward, done, info can't be included

    def close(self):
        pass


if __name__ == '__main__':

    rospy.init_node('wp3_gym_env', anonymous=True)
    print("start")

    wp3_env = wp3Env()
    check_env(wp3_env, warn=True)

    # init env
    obs = wp3_env.reset()

    # run model
    n_steps = 200
    for step in range(n_steps):
        # action, _states = model.predict(obs)
        action = wp3_env.action_space.sample()

        obs, rewards, done, info = wp3_env.step(action)

        time.sleep(0.1)
