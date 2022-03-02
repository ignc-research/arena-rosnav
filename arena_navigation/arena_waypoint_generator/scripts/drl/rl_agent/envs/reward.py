#! /usr/bin/env python3
import numpy as np
from typing import Tuple
import rospy
import scipy.spatial
import math

from geometry_msgs.msg import Pose2D, PoseStamped

class Reward():
    def __init__(
        self, 
        safe_dist: float,
        extended_eval: bool,
        planing_horizon: float,
        goal_radius: float = 0.7,
        rule: str = "rule_00",
        length: int=0,
    ):
        self.length = length
        self.curr_reward = 0
        self.info = {}
        self.goal_radius = goal_radius
        self._robot_radius = rospy.get_param("radius") + 0.075
        self.last_goal_dist = None
        self.last_dist_to_path = None
        self.kdtree = None
        self._curr_dist_to_path = None
        self._extended_eval = extended_eval
        if safe_dist is None:
            self.safe_dist = self._robot_radius + 0.3
        else:
            self.safe_dist = safe_dist

        self.subgoal_pose = Pose2D()
        self.planing_horizon = planing_horizon

        self._cal_funcs = {
            "rule_00": Reward._cal_reward_rule_00,
        }
        self.cal_func = self._cal_funcs[rule]

        self.counter = 0

    def reset_reward_(self):
        self.last_goal_dist = None
        self.last_dist_to_path = None
        self.kdtree = None
        self._curr_dist_to_path = None

    def reset_reward(self):
        self.curr_reward = 0
        self.info = {}

    def get_reward(self, laser_scan: np.ndarray,  goal_in_robot_frame: Tuple[float, float], *args, **kwargs):
        self.reset_reward()
        self.cal_func(self, laser_scan, goal_in_robot_frame, *args, **kwargs)
        return self.curr_reward, self.info

    def _cal_reward_rule_00(self, laser_scan: np.ndarray, goal_in_robot_frame: Tuple[float, float], *args, **kwargs):
        self.subgoal_pose = kwargs["subgoal"]
        self.goal_pose = kwargs["goal"]
        self.robot_pose = kwargs["robot_pose"]
        actions = kwargs["actions"]

        try:
            self._reward_goal_reached(goal_in_robot_frame, reward=17.5)
            self._reward_collision(laser_scan, punishment=10)
            self._reward_safe_dist(laser_scan, punishment=0.25)
            self._reward_on_global_plan(actions, reward=0.05)
            self._reward_subgoal_reached(self.subgoal_pose, self.robot_pose, reward=0.25)
            self._reward_stop_too_long(punishment = 7.5)
        except:
            pass

    def _reward_goal_reached(self, goal_in_robot_frame, reward: float = 17.5):
        if goal_in_robot_frame[0] < self.goal_radius:
            self.curr_reward = reward
            self.info["is_done"] = True
            self.info["done_reason"] = 2
            self.info["is_success"] = 1
        else:
            self.info["is_done"] = False        

    def _reward_collision(self, laser_scan: np.ndarray, punishment: float = 10):
        if np.min(laser_scan) <= self._robot_radius:
            self.curr_reward -= punishment

            if not self._extended_eval:
                self.info["is_done"] = True
                self.info["done_reason"] = 1
                self.info["is_success"] = 0
            else:
                self.info["crash"] = True

    def _reward_safe_dist(self, laser_scan: np.ndarray, punishment: float = 0.25):
        if np.min(laser_scan) < self.safe_dist:
            self.curr_reward -= punishment

            if self._extended_eval:
                self.info["safe_dist"] = True

    def _reward_on_global_plan(self, actions, reward: float = 0.05, punishment: float = 0.075):
        if actions[0] == int(self.length/2):
            self.curr_reward += reward
        else:
            self.curr_reward -= punishment

    def _reward_subgoal_reached(self, subgoal: Pose2D, robot: Pose2D, reward: float = 0.25):
        distance = math.hypot(robot.x - subgoal.x, robot.y - subgoal.y)
        if distance < self.goal_radius/2:
            self.counter += 1
        else:
            self.counter = 0

        if self.counter == 1:
            self.curr_reward += reward    

    def _reward_stop_too_long(self, punishment: float = 7.5):
        if self.counter > 20:
            self.curr_reward -= punishment
            self.info["is_done"] = True
            self.info["done_reason"] = 3
            self.info["is_success"] = 0