import math

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D


class Logger:

    def __init__(self, extended_eval: bool, is_evaluation: bool, max_steps_per_episode: int, number_models: int,
                 all_in_one_planner_frequency: int, model_names: [str]):
        self._extended_eval = extended_eval
        self._evaluation = is_evaluation
        self._max_steps_per_episode = max_steps_per_episode
        self._number_models = number_models
        self._model_names = model_names

        # for extended eval
        self._action_frequency = (1 / rospy.get_param("/robot_action_rate")) * all_in_one_planner_frequency
        self._last_robot_pose = None
        self._distance_travelled = 0
        self._safe_dist_counter = 0
        self._collisions = 0
        self._in_crash = False
        self._actions_close_obst_dist = []
        self._actions_medium_obst_dist = []
        self._actions_large_obst_dist = []
        self._actions = []

        self._episode_reward = 0

        self._last_actions_switch = np.zeros(shape=(self._number_models,))
        self._last_actions_switch_close_obst_dist = np.zeros(shape=(self._number_models,))
        self._last_actions_switch_medium_obst_dist = np.zeros(
            shape=(self._number_models,))
        self._last_actions_switch_large_obst_dist = np.zeros(shape=(self._number_models,))

    def get_step_info(self, done: bool, reward_info: dict, obs_dict: dict, reward, action: int, current_iteration: int):
        # extended eval info
        if self._extended_eval:
            self._update_eval_statistics(obs_dict, reward_info, reward, action, current_iteration)

        info = {}

        if done:
            info['done_reason'] = reward_info['done_reason']
            info['is_success'] = reward_info['is_success']

        if current_iteration > self._max_steps_per_episode:
            info['done_reason'] = 0
            info['is_success'] = 0

        if done and self._evaluation:
            print("Done reason: " + str(info['done_reason']) + " - Is success: " + str(
                info['is_success']) + " - Iterations: " + str(current_iteration) + " - Collisions: " + str(
                self._collisions) + " - Episode reward: " + str(self._episode_reward))

        # for logging
        if self._extended_eval:
            info['local_planner_comp_time'] = reward_info['local_planner_comp_time']
            if done:
                info['global_path_reward'] = reward_info['global_path_reward']
                info['collisions'] = self._collisions
                info['distance_travelled'] = round(self._distance_travelled, 2)
                info['time_safe_dist'] = self._safe_dist_counter * self._action_frequency
                info['time'] = current_iteration * self._action_frequency

                info['action_change_prob'] = self.get_switching_prob(self._actions)
                info['action_iterations'] = len(self._actions)
                info['model_distribution'] = self._last_actions_switch / np.sum(self._last_actions_switch)

                if np.sum(self._last_actions_switch_close_obst_dist) != 0:
                    info['model_distribution_close_obst_dist'] = self._last_actions_switch_close_obst_dist / np.sum(
                        self._last_actions_switch_close_obst_dist)
                    info['action_change_prob_close_obst_dist'] = self.get_switching_prob(self._actions_close_obst_dist)
                    info['action_iterations_close_obst_dist'] = len(self._actions_close_obst_dist)
                else:
                    info['model_distribution_close_obst_dist'] = self._last_actions_switch_close_obst_dist
                    info['action_change_prob_close_obst_dist'] = -1.0
                    info['action_iterations_close_obst_dist'] = 0

                if np.sum(self._last_actions_switch_medium_obst_dist) != 0:
                    info['model_distribution_medium_obst_dist'] = self._last_actions_switch_medium_obst_dist / np.sum(
                        self._last_actions_switch_medium_obst_dist)
                    info['action_change_prob_medium_obst_dist'] = self.get_switching_prob(self._actions_medium_obst_dist)
                    info['action_iterations_medium_obst_dist'] = len(self._actions_medium_obst_dist)
                else:
                    info['model_distribution_medium_obst_dist'] = self._last_actions_switch_medium_obst_dist
                    info['action_change_prob_medium_obst_dist'] = -1.0
                    info['action_iterations_medium_obst_dist'] = 0

                if np.sum(self._last_actions_switch_large_obst_dist) != 0:
                    info['model_distribution_large_obst_dist'] = self._last_actions_switch_large_obst_dist / np.sum(
                        self._last_actions_switch_large_obst_dist)
                    info['action_change_prob_large_obst_dist'] = self.get_switching_prob(
                        self._actions_large_obst_dist)
                    info['action_iterations_large_obst_dist'] = len(self._actions_large_obst_dist)
                else:
                    info['model_distribution_large_obst_dist'] = self._last_actions_switch_large_obst_dist
                    info['action_change_prob_large_obst_dist'] = -1.0
                    info['action_iterations_large_obst_dist'] = 0

                if self._evaluation:
                    print("Model distribution: " + str(info['model_distribution']) + " - " + str(self._model_names))

        return info, self._in_crash

    def reset(self):
        # extended eval info
        if self._extended_eval:
            self._last_robot_pose = None
            self._distance_travelled = 0
            self._safe_dist_counter = 0
            self._collisions = 0
            self._episode_reward = 0
            self._last_actions_switch = np.zeros(shape=(self._number_models,))
            self._last_actions_switch_close_obst_dist = np.zeros(shape=(self._number_models,))
            self._last_actions_switch_medium_obst_dist = np.zeros(shape=(self._number_models,))
            self._last_actions_switch_large_obst_dist = np.zeros(shape=(self._number_models,))
            self._actions_close_obst_dist = []
            self._actions_medium_obst_dist = []
            self._actions_large_obst_dist = []
            self._actions = []

    def _update_eval_statistics(self, obs_dict: dict, reward_info: dict, reward: float, action: int,
                                current_iteration: int):
        """
        Updates the metrics for extended eval mode

        param obs_dict (dict): observation dictionary from ObservationCollector.get_observations(),
            necessary entries: 'robot_pose'
        param reward_info (dict): dictionary containing information returned from RewardCalculator.get_reward(),
            necessary entries: 'crash', 'safe_dist'
        """
        # distance travelled
        if self._last_robot_pose is not None:
            self._distance_travelled += self.get_distance(
                self._last_robot_pose, obs_dict['robot_pose'])

        # collision detector
        if 'crash' in reward_info:
            if reward_info['crash'] and not self._in_crash:
                self._collisions += 1
                # when crash occures, robot strikes obst for a few consecutive timesteps
                # we want to count it as only one collision
                self._in_crash = True
        else:
            self._in_crash = False

        # safe dist detector
        if 'safe_dist' in reward_info:
            if reward_info['safe_dist']:
                self._safe_dist_counter += 1

        self._last_robot_pose = obs_dict['robot_pose']

        self._last_actions_switch[action] += 1
        min_obst_dist = np.min(obs_dict['scan_dynamic'])

        self._actions.append(action)
        if min_obst_dist < 1.2:
            self._last_actions_switch_close_obst_dist[action] += 1
            self._actions_close_obst_dist.append(action)
        elif min_obst_dist < 2.5:
            self._last_actions_switch_medium_obst_dist[action] += 1
            self._actions_medium_obst_dist.append(action)
        else:
            self._last_actions_switch_large_obst_dist[action] += 1
            self._actions_large_obst_dist.append(action)

        self._episode_reward += reward * (0.99 ** current_iteration)

    @staticmethod
    def get_distance(pose_1: Pose2D, pose_2: Pose2D):
        return math.hypot(pose_2.x - pose_1.x, pose_2.y - pose_1.y)

    @staticmethod
    def get_switching_prob(action_sequence: list):
        actions_np = np.array(action_sequence)
        # count changes starting from second element
        changes = np.where(actions_np[1:-1] != actions_np[2:])[0].size
        prob = float(changes) / len(action_sequence)
        return prob
