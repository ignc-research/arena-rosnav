import math
import os.path

import numpy as np
import rospkg
import torch
from rl_agent.envs.all_in_one_models.arena_ros import fc
from rl_agent.envs.all_in_one_models.model_base_class import ModelBase


class ArenaRosAgent(ModelBase):

    def __init__(self):
        observation_info = {'goal_in_robot_frame': True,
                            'laser_scan': True}
        super(ArenaRosAgent, self).__init__(observation_info, "arena_ros")

        # Define parameters
        self._action_space = {0: [0.3, 0], 1: [0.15, 0.75], 2: [0.15, -0.75], 3: [0.0, 1.5], 4: [0.0, -1.5]}
        self._beam_num = 360
        base_dir = rospkg.RosPack().get_path('arena_local_planner_drl')
        model_name = "dqn_agent_best_fc_l2.dat"
        nn_parameters_path = os.path.join(base_dir, 'rl_agent', 'envs', 'all_in_one_models', 'arena_ros', model_name)

        number_discrete_actions = len(self._action_space.keys())
        num_observations = 362

        # Load neural network
        self._net = fc.FC_DQN(num_observations, number_discrete_actions)
        self._net.train(False)  # set training mode to false to deactivate dropout layer
        self._net.load_state_dict(torch.load(nn_parameters_path, map_location=torch.device('cpu')));

    def get_next_action(self, observation_dict) -> np.ndarray:
        # goal
        sub_goal = observation_dict['goal_in_robot_frame']
        angle = sub_goal[1] + math.pi
        angle = np.arctan2(np.sin(angle), np.cos(angle))  # normalize angle
        angle = math.degrees(angle)
        distance = sub_goal[0] + 0.5

        # lidar scan
        scan = observation_dict['laser_scan']
        scan[:] = self._shift_scan(scan)

        observation = np.hstack([[distance, angle], scan])

        state_v = torch.FloatTensor([observation]).to('cpu')
        q_vals_v = self._net(state_v)

        # select action with max q value
        _, act_v = torch.max(q_vals_v, dim=1)

        action_discrete = int(act_v.item())
        return np.array(self._action_space[action_discrete])

    def wait_for_agent(self):
        return True

    def reset(self):
        pass

    def _shift_scan(self, scan: np.ndarray):
        # old angle: {min: 0, max: 6.28318977356, increment: 0.0175019223243}
        # new angle: {min: -1.5707963267948966, max: 4.694936014, increment:  0.017453292}
        sub_array = np.hsplit(scan, 4)
        new_scan = np.concatenate((sub_array[3], sub_array[0], sub_array[1], sub_array[2]))
        return new_scan
