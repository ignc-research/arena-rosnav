
#Aiming his file: subscribe the supervised learning nessesary data and store them as standard files
#outline of tasks
#done: # 1. action timeslot and observation timeslot are aligned.
#done  # 2. we should subscribe the keyboard controlled robots states and save as file (csv dosen't work!!, switch to h5py)(in this file)
#done  # 3. we should write a customer PPO which will be used to train ground truth acton-states pair (in pre_training_policies.py)
#done  # 4. write a customer MLP policy which is inherited from ActorCriticPolicy used to initialize ILStateCollectorEnv.
#done  # 5. get clear how to spawn random goal and map scenarios on the rviz and design an automative process to train the whole algo. (remember to add robot odom info)
#done  # 6. after a fixed trials and finishing supervised learning we should switch into drl training
#done  # 7. research reward function
#done  # 8. augmentate features to make sure agent be interested in goal.
#TODO  # 9. give a better demonstration or use planner to collect datasets
#TODO  # 10. mutiprocessing training.
#done  # * decoupled from flatland env

import time
import os
import numpy as np
import yaml
import h5py
from random import randint

from task_generator.tasks import get_predefined_task
from task_generator.tasks import ABSTask
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import rospy
import rospkg

from operator import is_
import gym
from gym import spaces
from gym.spaces import space
from typing import Union
from stable_baselines3.common.env_checker import check_env
from rl_agent.utils.observation_collector import ObservationCollector
from rl_agent.utils.reward import RewardCalculator
from rl_agent.utils.debug import timeit
from task_generator.tasks import ABSTask
import numpy as np
from flatland_msgs.srv import StepWorld, StepWorldRequest


key_mapping = { 'w': [ 0, 1.0], 'x': [ 0, -0.5],
        'a': [ 0.95, 0.25], 'd': [-0.95, 0.25],
        'q': [1.5, 0], 'e': [ -1.5, 0], 's': [ 0, 0]}

class ILStateCollectorEnv(gym.Env):
    def __init__(self,
                task: ABSTask,
                robot_yaml_path: str,
                settings_yaml_path: str,
                reward_fnc: str,
                is_action_space_discrete,
                is_state_collector,
                safe_dist: float = None,
                goal_radius: float = 0.1,
                max_steps_per_episode=100):
        super(ILStateCollectorEnv, self).__init__()
        ########### from flatland env #############
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
            robot_radius=self._robot_radius, safe_dist=1.1*self._robot_radius, goal_radius=goal_radius, rule=reward_fnc)
        ########### from flatland env #############

        # action agent publisher
        self.agent_action_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # service clients
        self._is_train_mode = rospy.get_param("train_mode")
        if self._is_train_mode:
            self._service_name_step = '/step_world'
            self._sim_step_client = rospy.ServiceProxy(
            self._service_name_step, StepWorld)
        self.task = task
        self._steps_curr_episode = 0
        self._max_steps_per_episode = max_steps_per_episode
        # action subscriber
        self._action_subscriber = rospy.Subscriber('keys', String, self._get_action_cb, queue_size=1)
        self.is_state_collector = is_state_collector
        self._curr_action = 6
        self._curr_cmd_vel = [0.,0.]
        self._current_reward = 0.
        self.observation_collector.register_reward(self._current_reward)
        self._absolute_path = rospkg.RosPack().get_path('arena_local_planner_il')
        self.reset()
        if is_state_collector:
            self._recording_indicator = 0.
        else:
            self._epi_reward = 0.
 

    def _get_action_cb(self, data:String):
        '''
        mapping from cmd_vel to discrete or continuous action,
        and save as a class variable.
        self._discrete_actions is a list, each element is a dict with the keys ["name", 'linear','angular'] from flatland_gym_env.py
        '''
        if data.data in key_mapping.keys():
            self._curr_cmd_vel = key_mapping[data.data]
        linear_vel = self._curr_cmd_vel[1]
        angular_vel = self._curr_cmd_vel[0]
        current_time = rospy.get_time()
        delta = 0.05 # velocity tolerance
        if self._is_action_space_discrete:
            if linear_vel > (key_mapping['w'][1] - delta):
                self._curr_action = 0
            elif linear_vel < (key_mapping['x'][1] + delta):
                self._curr_action = 1
            elif linear_vel > key_mapping['a'][1] - delta and angular_vel > key_mapping['a'][0] - delta:
                self._curr_action = 2
            elif linear_vel > key_mapping['d'][1] - delta and angular_vel < key_mapping['a'][0] + delta:
                self._curr_action = 3
            elif angular_vel > key_mapping['q'][0] - delta:
                self._curr_action = 4
            elif angular_vel < key_mapping['e'][0] + delta:
                self._curr_action = 5
            elif np.abs(angular_vel) < delta and np.abs(linear_vel) < delta:
                self._curr_action = 6
            else:
                self._curr_action = self._curr_action
        else:
            self._curr_cmd_vel = [linear_vel, angular_vel]
        
    def step(self, action):
        if not self.is_state_collector:
            self._pub_action(action)
        else:
            if self._is_action_space_discrete:
                self._pub_action(self._curr_action)
            else:
                self._pub_action(action)
        #todo add time alignment
        start_time = rospy.get_time()
        self._steps_curr_episode += 1
        # wait for new observations
        self.observation_collector.register_reward(self._current_reward)
        merged_obs, obs_dict = self.observation_collector.get_observations()

        states_time_slot = rospy.get_time()
        states_with_time = (states_time_slot, merged_obs)
        # calculate reward (don't need reward here)
        reward, reward_info = self.reward_calculator.get_reward(
            obs_dict['laser_scan'], obs_dict['goal_in_robot_frame'])
        self._current_reward = reward
        done = reward_info['is_done']
        # show the reward
        if not self.is_state_collector and not done:
            self._epi_reward += reward
        if not self.is_state_collector and done:
            self._epi_reward += reward
            print('current episode reward: %.3f' % self._epi_reward)
            self._epi_reward = 0.
        # info
        info = {}
        if done:
            info['done_reason'] = reward_info['done_reason']
        else:
            if self._steps_curr_episode == self._max_steps_per_episode:
                done = True
                print('current episode reward: %.3f' % self._epi_reward)
                self._epi_reward = 0.
                info['done_reason'] = 0

        if self.is_state_collector:
            # avoid recording one state twice
            if self._recording_indicator != action[0]:
                self._training_data_record(states_with_time, action)
                self._recording_indicator = action[0]
            # rechead the goal, reset task
            if done:
                self.task.reset()
            return states_with_time, reward, done, info # merged_obs.shape: (time_slot, numpy.ndarray(361,))
        else:
            return merged_obs, reward, done, info

    def _training_data_record(self, states, action):
        # Record the data as numpy array in HDF5
        if self._steps_curr_episode <= 1:
            self._f_action = h5py.File(self._absolute_path + '/data/action.hdf5', "w")
            self._f_state = h5py.File(self._absolute_path + '/data/state.hdf5', "w")
            
        self._f_action.create_dataset(str(action[0]), data=np.array(action))
        self._f_state.create_dataset(str(states[0]), data=np.array(states[1]))

    def close_file_writer(self):

        self._f_state.close()
        self._f_action.close()
        rospy.loginfo("h5py writer have been shut down.")

    def reset(self):
        self.agent_action_pub.publish(Twist())
        if self._is_train_mode:
            self._sim_step_client()
        self.task.reset()
        self.reward_calculator.reset()
        self._steps_curr_episode = 0
        obs, _ = self.observation_collector.get_observations()
        return obs  # reward, done, info can't be included

    def get_action(self):
        current_time = rospy.get_time()
        if self._is_action_space_discrete:
            action_with_time = (current_time, self._curr_action)
        else:
            action_with_time = (current_time, self._curr_cmd_vel)
        return action_with_time

    ################# from flatland env ####################### 
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
            action_msg.linear.x = action[1][0]
            action_msg.angular.z = action[1][1]
        self.agent_action_pub.publish(action_msg)


    def close(self):
        pass

if __name__ == '__main__':
    task = get_predefined_task()
    models_folder_path = rospkg.RosPack().get_path('simulator_setup')
    arena_local_planner_drl_folder_path = rospkg.RosPack().get_path('arena_local_planner_drl')
    
    rospy.init_node('il_state_collector')

    env = ILStateCollectorEnv(task, os.path.join(models_folder_path,'robot','myrobot.model.yaml'),
                    os.path.join(arena_local_planner_drl_folder_path,'configs','default_settings.yaml'),"rule_00", True, True,
                    max_steps_per_episode=600
                  )

    # temporary test
    n_step = 30000
    for time_step in range(n_step):
        action = env.get_action()
        merged_obs_with_time, _, done, _ = env.step(action)
        rospy.loginfo("time_step: {2} action_time: {0} obs_time:  {1} ".format(action[0], merged_obs_with_time[0], time_step))

    env.close_file_writer()