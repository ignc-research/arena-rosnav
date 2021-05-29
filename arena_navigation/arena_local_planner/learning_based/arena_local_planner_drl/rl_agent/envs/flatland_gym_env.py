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
from rl_agent.utils.CSVWriter import CSVWriter
from rl_agent.utils.reward import RewardCalculator
from rl_agent.utils.debug import timeit
from task_generator.tasks import *
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from flatland_msgs.srv import StepWorld, StepWorldRequest
from std_msgs.msg import Bool
import time

class Actions():
    # Define 11 choices of actions to be:
    # [v_pref,      [-pi/6, -pi/12, 0, pi/12, pi/6]]
    # [0.5*v_pref,  [-pi/6, 0, pi/6]]
    # [0,           [-pi/6, 0, pi/6]]
    def __init__(self):
        self.actions = np.mgrid[1.0:1.1:0.5, -np.pi/6:np.pi/6+0.01:np.pi/12].reshape(2, -1).T
        self.actions = np.vstack([self.actions,np.mgrid[0.5:0.6:0.5, -np.pi/6:np.pi/6+0.01:np.pi/6].reshape(2, -1).T])
        self.actions = np.vstack([self.actions,np.mgrid[0.0:0.1:0.5, -np.pi/6:np.pi/6+0.01:np.pi/6].reshape(2, -1).T])
        self.num_actions = len(self.actions)

class FlatlandEnv(gym.Env):
    """Custom Environment that follows gym interface"""

    def __init__(self, 
                 ns: str,  
                 robot_yaml_path: str, 
                 settings_yaml_path: str, 
                 reward_fnc: str, 
                 is_action_space_discrete, 
                 safe_dist: float = None, 
                 goal_radius: float = 0.1, 
                 max_steps_per_episode=100, 
                 train_mode: bool = True, 
                 debug: bool = False,
                 task_mode: str = "staged",
                 PATHS: dict = dict(),
                 *args, **kwargs):
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
        super(FlatlandEnv, self).__init__()
    

        self.ns = ns
        try:
            # given every environment enough time to initialize, if we dont put sleep,
            # the training script may crash.
            if ns.split("_")[1]!='sim':
                ns_int = int(ns.split("_")[1])
                time.sleep(ns_int*2)
            # else:ery environment enough time to initialize, if we dont put sleep,
            # the tra
            #     time.sleep(3)
        except Exception:
            rospy.logwarn(f"Can't not determinate the number of the environment, training script may crash!")
            pass

        # process specific namespace in ros system
        if ns is not None or ns !="":
            self.ns_prefix = '/'+ns + '/'
        else:
            self.ns_prefix = '/'
        
        if not debug:
            if train_mode:
                rospy.init_node(f'train_env_{self.ns}', disable_signals=False)
            else:
                rospy.init_node(f'eval_env_{self.ns}', disable_signals=False)

        # Define action and observation space
        # They must be gym.spaces objects
        self._is_train_mode = rospy.get_param("/train_mode")
        self._is_action_space_discrete = is_action_space_discrete
        
        self.setup_by_configuration(robot_yaml_path, settings_yaml_path)
        
        # set rosparam
        rospy.set_param("/laser_num_beams", self._laser_num_beams)

        # instantiate task manager
        self.task = get_predefined_task(
            ns, mode=task_mode, start_stage=kwargs['curr_stage'], PATHS=PATHS)
        # observation collector
        num_humans=self.task.obstacles_manager.num_humans
        num_robo_obstacles=self.task.obstacles_manager.num_robo_obstacles
        self.observation_collector = ObservationCollector(
            self.ns,kwargs['curr_stage'] )
        self.observation_collector.setRobotSettings(self._laser_num_beams, self._laser_max_range,
                                                                                                    self.laser_angle_min, self.laser_angle_max, 
                                                                                                    self.laser_angle_increment)
        self.observation_collector.setObservationSpace()
        self.observation_space = self.observation_collector.get_observation_space()
        #csv writer
        self.csv_writer=CSVWriter()

        # reward calculator
        if safe_dist is None:
            safe_dist = 1.6*self._robot_radius

        self.goal_radius=goal_radius

        self.reward_calculator = RewardCalculator(
            robot_radius=self._robot_radius, safe_dist=1.5*self._robot_radius, goal_radius=goal_radius, rule=reward_fnc)

        # action agent publisher
        if self._is_train_mode:
            self.agent_action_pub = rospy.Publisher(f'{self.ns_prefix}cmd_vel', Twist, queue_size=1)
        else:
            self.agent_action_pub = rospy.Publisher(f'{self.ns_prefix}cmd_vel_pub', Twist, queue_size=1)

        # service clients
        if self._is_train_mode:
            self._service_name_step = f'{self.ns_prefix}step_world'
            self._sim_step_client = rospy.ServiceProxy(
            self._service_name_step, StepWorld)
        


        self._steps_curr_episode = 0
        self._episode = 0
        self._max_steps_per_episode = max_steps_per_episode
        #store the obeservations from the last step for spawning the robot
        self.last_obs_dict=None
 
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
                                'radius', 0.3)*1.05
                        if footprint['radius']:
                            self._robot_radius = footprint['radius']*1.05
            # get laser related information
            for plugin in robot_data['plugins']:
                if plugin['type'] == 'Laser':
                    laser_angle_min = plugin['angle']['min']
                    laser_angle_max = plugin['angle']['max']
                    laser_angle_increment = plugin['angle']['increment']
                    self._laser_num_beams = int(
                        round((laser_angle_max-laser_angle_min)/laser_angle_increment)+1)
                    self._laser_max_range = plugin['range']
                    self.laser_angle_min = laser_angle_min
                    self.laser_angle_max = laser_angle_max
                    self.laser_angle_increment = laser_angle_increment

        with open(settings_yaml_path, 'r') as fd:
            setting_data = yaml.safe_load(fd)
            if self._is_action_space_discrete:
                # self._discrete_actions is a list, each element is a dict with the keys ["name", 'linear','angular']
                self._discrete_acitons = setting_data['robot']['discrete_actions']
                self.action_space = spaces.Discrete(
                    len(self._discrete_acitons))
                # self._discrete_acitons=Actions().actions
                # self.action_space = spaces.Discrete(Actions().num_actions)
            else:
                linear_range = setting_data['robot']['continuous_actions']['linear_range']
                angular_range = setting_data['robot']['continuous_actions']['angular_range']
                self.action_space = spaces.Box(low=np.array([linear_range[0], angular_range[0]]),
                                               high=np.array(
                                                   [linear_range[1], angular_range[1]]),
                                               dtype=np.float)

    def _pub_action(self, action):
        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.angular.z = action[1]
        self.agent_action_pub.publish(action_msg)

    def _translate_disc_action(self, action):
        new_action = np.array([])
        # new_action = np.append(new_action, self._discrete_acitons[action][0])
        # new_action = np.append(new_action, self._discrete_acitons[action][1])
        new_action = np.append(new_action, self._discrete_acitons[action]['linear'])
        new_action = np.append(new_action, self._discrete_acitons[action]['angular'])    
            
        return new_action

    def step(self, action):
        """
        done_reasons:   0   -   exceeded max steps
                        1   -   collision with obstacle
                        2   -   goal reached
                        >3   -   too close to human
        """
        if self._is_action_space_discrete:
            action = self._translate_disc_action(action)
        self._pub_action(action)
        #print(f"Linear: {action[0]}, Angular: {action[1]}")
        self._steps_curr_episode += 1
        s = time.time()
        #tell the robot how long it has passed
        self.observation_collector.set_timestep(self._steps_curr_episode/self._max_steps_per_episode)
        # wait for new observations
        while  rospy.get_param("/_initiating_stage") == True : 
            print('################ waiting for _initiating_stage #######################')
            time.sleep(1)
        merged_obs, obs_dict = self.observation_collector.get_observations()


        # calculate reward
        reward, reward_info = self.reward_calculator.get_reward(
            obs_dict['laser_scan'], obs_dict['goal_in_robot_frame'], 
            obs_dict['human_obstacles_in_robot_frame'], obs_dict['robot_obstacles_in_robot_frame'],obs_dict["robot_velocity"],self._steps_curr_episode/self._max_steps_per_episode
        )
        done = reward_info['is_done']
        # print("cum_reward:  {}".format(reward))
        
        # info
        info = {}
        if done:
            info['done_reason'] = reward_info['done_reason']
            info['is_success'] = reward_info['is_success']
            self.reward_calculator.kdtree = None
            history_evaluation=self.reward_calculator.get_history_info()
            history_evaluation=[self._episode]+history_evaluation+[info['done_reason']]
            self.csv_writer.addData(np.array(history_evaluation))

        if self._steps_curr_episode > self._max_steps_per_episode:
            done = True
            info['done_reason'] = 0
            info['is_success'] = 0
            self.reward_calculator.kdtree = None
            history_evaluation=self.reward_calculator.get_history_info()
            history_evaluation=[self._episode]+history_evaluation+[info['done_reason']]
            self.csv_writer.addData(np.array(history_evaluation))

        self.last_obs_dict=obs_dict

        return merged_obs, reward, done, info

    def reset(self):
        # set task
        # regenerate start position end goal position of the robot and change the obstacles and ped accordingly
        self._episode += 1
        self.agent_action_pub.publish(Twist())
        if self._is_train_mode:
            self._sim_step_client()
        #reset start position end goal position  ped positions TODO : modify the reset mechnism
        self.task.reset(self.last_obs_dict, self._episode, self.goal_radius)
        #reset 
        # self.task.obstacles_manager.__move_all_peds(self._episode)
        self.reward_calculator.reset()
        self._steps_curr_episode = 0        
        self.observation_collector.set_timestep(0.0)
        obs, _ = self.observation_collector.get_observations()
        return obs  # reward, done, info can't be included

    def close(self):
        pass


if __name__ == '__main__':

    rospy.init_node('flatland_gym_env', anonymous=True, disable_signals=False)
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
