from typing import Tuple

import numpy as np
import rospy
import scipy.spatial
from geometry_msgs.msg import Pose2D


class RewardCalculator:
    def __init__(self,
                 robot_radius: float,
                 safe_dist: float,
                 goal_radius: float,
                 rule: str = 'rule_00',
                 global_planner_frequency: int = 5,
                 extended_eval: bool = False):
        """
        A class for calculating reward based various rules.


        :param safe_dist (float): The minimum distance to obstacles or wall that robot is in safe status.
                                  if the robot get too close to them it will be punished. Unit[ m ]
        :param goal_radius (float): The minimum distance to goal that goal position is considered to be reached.
        """
        self._is_train_mode = rospy.get_param("/train_mode")

        self.curr_reward = 0
        # additional info will be stored here and be returned alonge with reward.
        self.info = {}
        self.robot_radius = robot_radius
        self.goal_radius = goal_radius
        self.last_goal_dist = None
        self.last_dist_to_path = None
        self.last_action = None
        self.safe_dist = safe_dist
        self.extended_eval = extended_eval

        self._global_planner_frequency = global_planner_frequency
        self.kdtree = None

        if self.extended_eval:
            self._collision_reward = 0
            self._goal_reached_reward = 0
            self.global_plan_reward = 0

        self._cal_funcs = {
            'rule_00': RewardCalculator._cal_reward_rule_00,
            'rule_01': RewardCalculator._cal_reward_rule_01,
            'rule_02': RewardCalculator._cal_reward_rule_02,
            'rule_03': RewardCalculator._cal_reward_rule_03,
            'rule_04': RewardCalculator._cal_reward_rule_04,
            'rule_06': RewardCalculator._cal_reward_rule_06,
            'rule_07': RewardCalculator._cal_reward_rule_07
        }
        self.cal_func = self._cal_funcs[rule]

    def reset(self):
        """
        reset variables related to the episode
        """
        self.last_goal_dist = None
        self.last_dist_to_path = None
        self.last_action = None
        self.kdtree = None

        if self.extended_eval:
            self._collision_reward = 0
            self._goal_reached_reward = 0
            self.global_plan_reward = 0

    def _reset(self):
        """
        reset variables related to current step
        """
        self.curr_reward = 0
        self.info = {}

    def get_reward(self,
                   laser_scan: np.ndarray,
                   goal_in_robot_frame: Tuple[float, float],
                   *args, **kwargs):
        """
        Returns reward and info to the gym environment.

        :param laser_scan (np.ndarray): laser scan data
        :param goal_in_robot_frame (Tuple[float,float]): position (rho, theta) of the goal in robot frame (Polar coordinate)
        """
        self._reset()
        self.cal_func(self, laser_scan, goal_in_robot_frame, *args, **kwargs)
        return self.curr_reward, self.info

    def set_global_planner_frequency(self, freq: int):
        self._global_planner_frequency = freq

    def _cal_reward_rule_00(self,
                            laser_scan: np.ndarray,
                            goal_in_robot_frame: Tuple[float, float],
                            *args, **kwargs):
        self._reward_goal_reached(
            goal_in_robot_frame)
        self._reward_safe_dist(
            laser_scan, punishment=0.25)
        self._reward_collision(
            laser_scan)
        self._reward_goal_approached(
            goal_in_robot_frame, reward_factor=0.3, penalty_factor=0.4)

    def _cal_reward_rule_01(self,
                            laser_scan: np.ndarray,
                            goal_in_robot_frame: Tuple[float, float],
                            *args, **kwargs):
        self._reward_distance_traveled(
            kwargs['action'], consumption_factor=0.0075)
        self._reward_goal_reached(
            goal_in_robot_frame, reward=15)
        self._reward_safe_dist(
            laser_scan, punishment=0.25)
        self._reward_collision(
            laser_scan, punishment=10)
        self._reward_goal_approached(
            goal_in_robot_frame, reward_factor=0.3, penalty_factor=0.4)

    def _cal_reward_rule_02(self,
                            laser_scan: np.ndarray,
                            goal_in_robot_frame: Tuple[float, float],
                            *args, **kwargs):
        self._reward_distance_traveled(
            kwargs['action'], consumption_factor=0.0075)
        self._reward_following_global_plan(
            kwargs['global_plan'], kwargs['robot_pose'])
        self._reward_goal_reached(
            goal_in_robot_frame, reward=15)
        self._reward_safe_dist(
            laser_scan, punishment=0.25)
        self._reward_collision(
            laser_scan, punishment=10)
        self._reward_goal_approached(
            goal_in_robot_frame, reward_factor=0.3, penalty_factor=0.4)

    def _cal_reward_rule_03(self,
                            laser_scan: np.ndarray,
                            goal_in_robot_frame: Tuple[float, float],
                            *args, **kwargs):
        self._reward_following_global_plan(
            kwargs['global_plan'], kwargs['robot_pose'], kwargs['action'])
        if laser_scan.min() > self.safe_dist:
            self._reward_distance_global_plan(
                kwargs['global_plan'], kwargs['robot_pose'], reward_factor=0.2, penalty_factor=0.3)
        else:
            self.last_dist_to_path = None
        self._reward_goal_reached(
            goal_in_robot_frame, reward=15)
        self._reward_safe_dist(
            laser_scan, punishment=0.25)
        self._reward_collision(
            laser_scan, punishment=10)
        self._reward_goal_approached(
            goal_in_robot_frame, reward_factor=0.3, penalty_factor=0.4)

    def _cal_reward_rule_04(self,
                            laser_scan: np.ndarray,
                            goal_in_robot_frame: Tuple[float, float],
                            *args, **kwargs):
        self._reward_abrupt_direction_change(
            kwargs['action'])
        self._reward_following_global_plan(
            kwargs['global_plan'], kwargs['robot_pose'], kwargs['action'])
        if laser_scan.min() > self.safe_dist:
            self._reward_distance_global_plan(
                kwargs['global_plan'], kwargs['robot_pose'], reward_factor=0.2, penalty_factor=0.3)
        else:
            self.last_dist_to_path = None
        self._reward_goal_reached(
            goal_in_robot_frame, reward=15)
        self._reward_safe_dist(
            laser_scan, punishment=0.25)
        self._reward_collision(
            laser_scan, punishment=10)
        self._reward_goal_approached(
            goal_in_robot_frame, reward_factor=0.3, penalty_factor=0.4)

    def _cal_reward_rule_06(self,
                            laser_scan: np.ndarray,
                            goal_in_robot_frame: Tuple[float, float],
                            *args, **kwargs):
        if not laser_scan.min() > self.safe_dist:
            self.last_dist_to_path = None
        self._reward_goal_reached(
            goal_in_robot_frame, reward=3)
        self._reward_safe_dist(
            laser_scan, punishment=0.02)
        self._reward_goal_approached(kwargs['sub_goal'], kwargs['new_global_plan'], reward_factor=0.05,
                                     penalty_factor=0.1)
        self._reward_collision(
            laser_scan, punishment=2)

    def _cal_reward_rule_07(self,
                            laser_scan: np.ndarray,
                            goal_in_robot_frame: Tuple[float, float],
                            *args, **kwargs):
        if not laser_scan.min() > self.safe_dist:
            self.last_dist_to_path = None
        scan_dynamic = kwargs['scan_dynamic']
        if np.min(scan_dynamic) < 1.4:
            # case 1: Obstacle avoidance
            self._reward_safe_dist(
                laser_scan, punishment=0.03)
            self._reward_goal_approached(kwargs['sub_goal'], kwargs['new_global_plan'], reward_factor=0.1,
                                         penalty_factor=0.0)
        else:
            # case 2: global path following
            self._reward_goal_approached(kwargs['sub_goal'], kwargs['new_global_plan'], reward_factor=0.1,
                                         penalty_factor=0.5)
            # Punish standing still
            self._standing_still(kwargs['action'], punishment=0.005)

        # check for final rewards
        self._reward_goal_reached(
            goal_in_robot_frame, reward=3)
        self._reward_collision(
            laser_scan, punishment=3)

    def _reward_goal_reached(self,
                             goal_in_robot_frame=Tuple[float, float],
                             reward: float = 15):
        """
        Reward for reaching the goal.

        :param goal_in_robot_frame (Tuple[float,float]): position (rho, theta) of the goal in robot frame (Polar coordinate)
        :param reward (float, optional): reward amount for reaching. defaults to 15
        """
        if goal_in_robot_frame[0] < self.goal_radius:
            if self.extended_eval:
                self._goal_reached_reward = reward
                self.info['global_path_reward'] = self.global_plan_reward

            self.curr_reward = reward
            self.info['is_done'] = True
            self.info['done_reason'] = 2
            self.info['is_success'] = 1

        else:
            self.info['is_done'] = False

    def _reward_goal_approached(self,
                                goal_in_robot_frame=Tuple[float, float],
                                new_global_plan=False,
                                reward_factor: float = 0.3,
                                penalty_factor: float = 0.5):
        """
        Reward for approaching the goal.

        :param goal_in_robot_frame (Tuple[float,float]): position (rho, theta) of the goal in robot frame (Polar coordinate)
        :param reward_factor (float, optional): positive factor for approaching goal. defaults to 0.3
        :param penalty_factor (float, optional): negative factor for withdrawing from goal. defaults to 0.5
        """
        if self.last_goal_dist is not None and not new_global_plan:
            # goal_in_robot_frame : [rho, theta]

            # higher negative weight when moving away from goal
            # (to avoid driving unnecessary circles when train in contin. action space)
            if (self.last_goal_dist - goal_in_robot_frame[0]) > 0:
                w = reward_factor
            else:
                w = penalty_factor
            reward = w * (self.last_goal_dist - goal_in_robot_frame[0])

            # print("reward_goal_approached:  {}".format(reward))
            self.curr_reward += reward

            if self.extended_eval:
                self.global_plan_reward += reward
        self.last_goal_dist = goal_in_robot_frame[0]

    def _reward_collision(self,
                          laser_scan: np.ndarray,
                          punishment: float = 10):
        """
        Reward for colliding with an obstacle.

        :param laser_scan (np.ndarray): laser scan data
        :param punishment (float, optional): punishment for collision. defaults to 10
        """
        if laser_scan.min() <= self.robot_radius:
            self.curr_reward -= punishment

            if self._is_train_mode:
                self.info['is_done'] = True
                self.info['done_reason'] = 1
                self.info['is_success'] = 0
                if self.extended_eval:
                    self.info['global_path_reward'] = self.global_plan_reward
                self.info['crash'] = True

    def _reward_safe_dist(self,
                          laser_scan: np.ndarray,
                          punishment: float = 0.15):
        """
        Reward for undercutting safe distance.

        :param laser_scan (np.ndarray): laser scan data
        :param punishment (float, optional): punishment for undercutting. defaults to 0.15
        """
        if laser_scan.min() < self.safe_dist:
            self.curr_reward -= punishment

            if self.extended_eval:
                self.info['safe_dist'] = True

    def _reward_not_moving(self,
                           action: np.ndarray = None,
                           punishment: float = 0.01):
        """
        Reward for not moving. Only applies half of the punishment amount
        when angular velocity is larger than zero.

        :param action (np.ndarray (,2)): [0] - linear velocity, [1] - angular velocity
        :param punishment (float, optional): punishment for not moving. defaults to 0.01
        """
        if action is not None and action[0] == 0.0:
            if action[1] == 0.0:
                self.curr_reward -= punishment
            else:
                self.curr_reward -= punishment / 2

    def _reward_distance_traveled(self,
                                  action: np.array = None,
                                  punishment: float = 0.01,
                                  consumption_factor: float = 0.005):
        """
        Reward for driving a certain distance. Supposed to represent "fuel consumption".

        :param action (np.ndarray (,2)): [0] - linear velocity, [1] - angular velocity
        :param punishment (float, optional): punishment when action can't be retrieved. defaults to 0.01
        :param consumption_factor (float, optional): weighted velocity punishment. defaults to 0.01
        """
        if action is None:
            self.curr_reward -= punishment
        else:
            lin_vel = action[0]
            ang_vel = action[1]
            reward = (lin_vel + (ang_vel * 0.001)) * consumption_factor
        self.curr_reward -= reward

    def _reward_distance_global_plan(self,
                                     global_plan: np.array,
                                     robot_pose: Pose2D,
                                     reward_factor: float = 0.1,
                                     penalty_factor: float = 0.15):
        """
        Reward for approaching/veering away the global plan. (Weighted difference between
        prior distance to global plan and current distance to global plan)

        :param global_plan: (np.ndarray): vector containing poses on global plan
        :param robot_pose (Pose2D): robot position
        :param reward_factor (float, optional): positive factor when approaching global plan. defaults to 0.1
        :param penalty_factor (float, optional): negative factor when veering away from global plan. defaults to 0.15
        """
        if global_plan is not None and len(global_plan) != 0:
            curr_dist_to_path, idx = self.get_min_dist2global_kdtree(
                global_plan, robot_pose)

            if self.last_dist_to_path is not None:
                if curr_dist_to_path < self.last_dist_to_path:
                    w = reward_factor
                else:
                    w = penalty_factor

                self.curr_reward += w * (self.last_dist_to_path - curr_dist_to_path)
            self.last_dist_to_path = curr_dist_to_path

    def _reward_following_global_plan(self,
                                      global_plan: np.array,
                                      robot_pose: Pose2D,
                                      action: np.array = None,
                                      dist_to_path: float = 0.5):
        """
        Reward for travelling on the global plan.

        :param global_plan: (np.ndarray): vector containing poses on global plan
        :param robot_pose (Pose2D): robot position
        :param action (np.ndarray (,2)): [0] = linear velocity, [1] = angular velocity
        :param dist_to_path (float, optional): applies reward within this distance
        """
        if global_plan is not None and len(global_plan) != 0 and action is not None:
            curr_dist_to_path, idx = self.get_min_dist2global_kdtree(
                global_plan, robot_pose)

            if curr_dist_to_path <= dist_to_path:
                self.curr_reward += 0.1 * action[0]

    def get_min_dist2global_kdtree(self,
                                   global_plan: np.array,
                                   robot_pose: Pose2D):
        """
        Calculates minimal distance to global plan using kd tree search.

        :param global_plan: (np.ndarray): vector containing poses on global plan
        :param robot_pose (Pose2D): robot position
        """
        if self.kdtree is None:
            self.kdtree = scipy.spatial.cKDTree(global_plan)

        dist, index = self.kdtree.query([robot_pose.x, robot_pose.y])
        return dist, index

    def _reward_abrupt_direction_change(self,
                                        action: np.array = None):
        """
        Applies a penalty when an abrupt change of direction occured.

        :param action: (np.ndarray (,2)): [0] = linear velocity, [1] = angular velocity
        """
        if self.last_action is not None:
            curr_ang_vel = action[1]
            last_ang_vel = self.last_action[1]

            vel_diff = abs(curr_ang_vel - last_ang_vel)
            self.curr_reward -= (vel_diff ** 4) / 2500
        self.last_action = action

    def _standing_still(self, action, punishment):
        if action[0] < 0.1:
            self.curr_reward -= punishment
