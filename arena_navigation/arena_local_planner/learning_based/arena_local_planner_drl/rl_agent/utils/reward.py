import numpy as np
from numpy.lib.utils import safe_eval
import rospy
from typing import Tuple

class RewardCalculator():
    def __init__(self, robot_radius: float, safe_dist:float, goal_radius:float, rule:str = 'rule_00' ):
        """A class for calculating reward based various rules.

        Args:
            safe_dist (float): The minimum distance to obstacles or wall that robot is in safe status.
                if the robot get too close to them it will be punished. Unit[ m ]
            goal_radius (float): The minimum distance to goal that goal position is considered to be reached. 
        """
        self.curr_reward = 0
        # additional info will be stored here and be returned alonge with reward.
        self.info = {}
        self.robot_radius = robot_radius
        self.goal_radius = goal_radius
        self.last_goal_dist = None
        self.safe_dist = safe_dist
        self._cal_funcs = {
            'rule_00': RewardCalculator._cal_reward_rule_00,
            'rule_01': RewardCalculator._cal_reward_rule_01
            }
        self.cal_func = self._cal_funcs[rule]

    def reset(self):
        """reset variables related to the episode
        """
        self.last_goal_dist = None

    def _reset(self):
        """reset variables related to current step
        """
        self.curr_reward = 0
        self.info = {}
    
    def get_reward(self, laser_scan:np.ndarray, goal_in_robot_frame: Tuple[float,float], *args, **kwargs):
        """

        Args:
            laser_scan (np.ndarray): 
            goal_in_robot_frame (Tuple[float,float]: position (rho, theta) of the goal in robot frame (Polar coordinate)  
        """

        self._reset()
        self.cal_func(self,laser_scan,goal_in_robot_frame,*args,**kwargs)
        return self.curr_reward, self.info


    def _cal_reward_rule_00(self, laser_scan: np.ndarray, goal_in_robot_frame: Tuple[float,float],*args,**kwargs):

        self._reward_goal_reached(goal_in_robot_frame)
        self._reward_safe_dist(laser_scan)
        self._reward_collision(laser_scan)
        self._reward_goal_approached(goal_in_robot_frame)
        

    def _cal_reward_rule_01(self, laser_scan: np.ndarray, goal_in_robot_frame: Tuple[float,float],*args,**kwargs):
        
        self._reward_goal_reached(goal_in_robot_frame)
        self._reward_safe_dist(laser_scan)
        self._reward_collision(laser_scan)
        self._reward_goal_approached2(goal_in_robot_frame)
        

    def _reward_goal_reached(self,goal_in_robot_frame, reward = 15):

        if goal_in_robot_frame[0] < self.goal_radius:
            self.curr_reward = reward
            self.info['is_done'] = True
            self.info['done_reason'] = 2
        else:
            self.info['is_done'] = False


    def _reward_goal_approached(self, goal_in_robot_frame,reward = 1, punishment = 0.0001):
        if self.last_goal_dist is not None:
            #goal_in_robot_frame : [rho, theta]
            """
            if goal_in_robot_frame[0] < self.last_goal_dist:
                self.curr_reward += reward
            else:
                self.curr_reward -=punishment
            """

            # if current goal distance shorter than last one, positive weighted reward - otherwise negative wegihted reward
            w = 0.25
            reward = round(w*(self.last_goal_dist - goal_in_robot_frame[0]), 3)
            
            # punishment for not moving
            if self.last_goal_dist == goal_in_robot_frame[0]:
                reward = -punishment
            self.curr_reward += reward
        self.last_goal_dist = goal_in_robot_frame[0]


    def _reward_goal_approached2(self, goal_in_robot_frame,reward = 1, punishment = 0.01):
        if self.last_goal_dist is not None:
            #goal_in_robot_frame : [rho, theta]

            # if current goal distance shorter than last one, positive weighted reward - otherwise negative wegihted reward
            w = 0.25
            reward = round(w*(self.last_goal_dist - goal_in_robot_frame[0]), 3)
            
            # higher negative weight when moving away from goal (to avoid driving unnecessary circles when train in contin. action space)
            if (self.last_goal_dist - goal_in_robot_frame[0]) > 0:
                w = 0.25
            elif (self.last_goal_dist - goal_in_robot_frame[0]) < 0:
                w = 0.4
            reward = round(w*(self.last_goal_dist - goal_in_robot_frame[0]), 3)

            # punishment for not moving
            if self.last_goal_dist == goal_in_robot_frame[0]:
                reward = -punishment
            self.curr_reward += reward
        self.last_goal_dist = goal_in_robot_frame[0]


    def _reward_collision(self,laser_scan, punishment = 10):
        if laser_scan.min() <= self.robot_radius:
            self.curr_reward -= punishment
            self.info['is_done'] = True
            self.info['done_reason'] = 1

    
    def _reward_safe_dist(self, laser_scan, punishment = 0.15):
        if laser_scan.min() < self.safe_dist:
            self.curr_reward -= punishment
    



        