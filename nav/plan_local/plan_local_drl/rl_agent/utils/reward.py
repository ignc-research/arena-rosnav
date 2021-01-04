import numpy as np
from numpy.lib.utils import safe_eval
import rospy
from typing import Tuple

class RewardCalculator():
    def __init__(self, safe_dist:float, goal_radius:float, rule:str = 'rule_00' ):
        """A class for calculating reward based various rules.

        Args:
            safe_dist (float): The minimum distance to obstacles or wall that robot is in safe status.
                if the robot get too close to them it will be punished. Unit[ m ]
            goal_radius (float): The minimum distance to goal that goal position is considered to be reached. 
        """
        self.curr_reward = 0
        # additional info will be stored here and be returned alonge with reward.
        self.info = {}
        self.goal_radius = goal_radius
        self.last_goal_dist = None
        self.safe_dist = safe_dist
        self._cal_funcs = {'rule_00': RewardCalculator._cal_reward_rule_00}
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

        self._reward_goal_appoached(goal_in_robot_frame)
        self._reward_goal_reached(goal_in_robot_frame)
        self._reward_laserscan_general(laser_scan)
    

    def _reward_goal_reached(self,goal_in_robot_frame, reward = 10):

        if goal_in_robot_frame[0] < self.goal_radius:
            self.curr_reward = reward
            self.info['is_done'] = True
        else:
            self.info['is_done'] = False

    def _reward_goal_appoached(self, goal_in_robot_frame,reward = 1, punishment = 0.2):
        if self.last_goal_dist is not None:
            #goal_in_robot_frame : [rho, theta] 
            if goal_in_robot_frame[0] < self.last_goal_dist:
                self.curr_reward += reward
            else:
                self.curr_reward -=punishment
        self.last_goal_dist = goal_in_robot_frame[0]

    def _reward_laserscan_general(self,laser_scan, punishment = 7):
        if laser_scan.min()<self.safe_dist:
            self.curr_reward -= punishment
    
    


        