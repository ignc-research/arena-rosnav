import copy
from collections import deque

import numpy as np
import rospkg
import torch
from rl_agent.envs.all_in_one_models.model_base_class import ModelBase
from rl_collision_avoidance.circle_world import StageWorld
from rl_collision_avoidance.model.net import CNNPolicy
from rl_collision_avoidance.model.ppo import generate_action_no_sampling


class RLCAAgent(ModelBase):

    def __init__(self):
        observation_info = {'goal_in_robot_frame_xy': True,
                            'laser_3': True,
                            'robot_twist': True}

        super(RLCAAgent, self).__init__(observation_info, "rlca")

        # Set parameters of env
        laser_hist = 3
        self._beam_num = 512
        obs_size = self._beam_num  # number of leaser beam

        # TODO make this a parameter
        self._action_bound = [[0, -1], [1, 1]]  # the limitation of velocity

        # Set env and agent policy
        self._env = StageWorld(obs_size, index=0, num_env=1)  # index 0 means first agent (for multiprocessing)
        trained_model_file = rospkg.RosPack().get_path('rl_collision_avoidance') + \
                             '/rl_collision_avoidance/policy/stage2.pth'

        policy = CNNPolicy(frames=laser_hist, action_space=2)
        policy.cpu()
        state_dict = torch.load(trained_model_file, map_location=torch.device('cpu'))
        policy.load_state_dict(state_dict)
        self._policy = policy

    def get_next_action(self, observation_dict) -> np.ndarray:
        # extract and transform input

        obs_laser_3 = observation_dict['laser_3']
        obs_laser_3_transformed = [self._transform_scan_data(obs_laser_3[:, 2]),
                                   self._transform_scan_data(obs_laser_3[:, 1]),
                                   self._transform_scan_data(obs_laser_3[:, 0])]

        obs_sub_goal = np.asarray(observation_dict['goal_in_robot_frame_xy'], dtype='float64')
        obs_twist = observation_dict['robot_twist']

        obs_stack = deque([obs_laser_3_transformed[0], obs_laser_3_transformed[1], obs_laser_3_transformed[2]])
        velocity = np.asarray([obs_twist.linear.x, obs_twist.angular.z], dtype='float64')

        obs_state_list = [[obs_stack, obs_sub_goal, velocity]]

        # get output
        _, scaled_action = generate_action_no_sampling(self._env, obs_state_list, self._policy, self._action_bound)
        action = scaled_action[0]
        # the maximum speed of cmd_vel 0.3
        action[0] = 0.3 * action[0]
        return np.asarray([action[0], action[1]])

    def wait_for_agent(self):
        return True

    def reset(self):
        pass

    def _transform_scan_data(self, scan):
        # Note: Taken from rl_collision_avoidance_tb3.py
        scan = copy.deepcopy(scan)
        sub_array = np.hsplit(scan,
                              4)  # adapt scan info when min and max angel equal to [-1.57,4.69] (rlca is [-3.14,3.14])
        scan = np.concatenate((sub_array[3], sub_array[0], sub_array[1], sub_array[
            2]))  # adapt scan info when min and max angel equal to [-1.57,4.69] (rlca is [-3.14,3.14])

        scan[np.isnan(scan)] = 6.0
        scan[np.isinf(scan)] = 6.0
        raw_beam_num = len(scan)
        sparse_beam_num = self._beam_num
        step = float(raw_beam_num) / sparse_beam_num
        sparse_scan_left = []
        index = 0.
        for x in range(int(sparse_beam_num / 2)):
            sparse_scan_left.append(scan[int(index)])
            index += step
        sparse_scan_right = []
        index = raw_beam_num - 1.
        for x in range(int(sparse_beam_num / 2)):
            sparse_scan_right.append(scan[int(index)])
            index -= step
        scan_sparse = np.concatenate((sparse_scan_left, sparse_scan_right[::-1]), axis=0)
        return scan_sparse / 6.0 - 0.5
