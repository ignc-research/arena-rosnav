import numpy as np
from numpy.lib.utils import safe_eval
import rospy
from geometry_msgs.msg import Pose2D
from typing import Tuple
import scipy.spatial
from rl_agent.utils.debug import timeit

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
        self.last_dist_to_path = np.inf
        self.safe_dist = safe_dist
        self._cal_funcs = {
            'rule_00': RewardCalculator._cal_reward_rule_00,
            'rule_01': RewardCalculator._cal_reward_rule_01,
            'rule_02': RewardCalculator._cal_reward_rule_02,
            'rule_03': RewardCalculator._cal_reward_rule_03
            }
        self.cal_func = self._cal_funcs[rule]
        self._tree_cache  = {'id':None,'tree':None}

    def reset(self):
        """reset variables related to the episode
        """
        self.last_goal_dist = None
        self.last_dist_to_path = np.inf

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
        self._reward_not_moving(goal_in_robot_frame)
        self._reward_safe_dist(laser_scan)
        self._reward_collision(laser_scan)
        self._reward_goal_approached(goal_in_robot_frame)
                

    def _cal_reward_rule_01(self, laser_scan: np.ndarray, goal_in_robot_frame: Tuple[float,float],*args,**kwargs):
        self._reward_goal_reached(goal_in_robot_frame)
        self._reward_not_moving(goal_in_robot_frame)
        self._reward_safe_dist(laser_scan)
        self._reward_collision(laser_scan)
        self._reward_goal_approached2(goal_in_robot_frame)
        

    def _cal_reward_rule_02(self, laser_scan: np.ndarray, goal_in_robot_frame: Tuple[float,float],*args,**kwargs):
        self._reward_distance_traveled(kwargs['action'])
        self._reward_goal_reached(goal_in_robot_frame, reward=15)
        self._reward_safe_dist(laser_scan, punishment=0.1)
        self._reward_collision(laser_scan, punishment=20)
        self._reward_goal_approached2(goal_in_robot_frame)


    def _cal_reward_rule_03(self, laser_scan: np.ndarray, goal_in_robot_frame: Tuple[float,float],*args,**kwargs):
        self._reward_distance_traveled(kwargs['action'])
        self._reward_global_plan(kwargs['global_plan'], kwargs['robot_pose'])
        self._reward_goal_reached(goal_in_robot_frame, reward=15)
        self._reward_safe_dist(laser_scan, punishment=0.2)
        self._reward_collision(laser_scan, punishment=10)
        self._reward_goal_approached2(goal_in_robot_frame)


    def _reward_goal_reached(self,goal_in_robot_frame, reward = 15):
        if goal_in_robot_frame[0] < self.goal_radius:
            self.curr_reward = reward
            self.info['is_done'] = True
            self.info['done_reason'] = 2
            self.info['is_success'] = 1
        else:
            self.info['is_done'] = False


    def _reward_goal_approached(self, goal_in_robot_frame):
        if self.last_goal_dist is not None:
            # goal_in_robot_frame : [rho, theta]
            # if current goal distance shorter than last one, positive weighted reward - otherwise negative wegihted reward
            w = 0.25
            reward = round(w*(self.last_goal_dist - goal_in_robot_frame[0]), 3)
            
            self.curr_reward += reward
        self.last_goal_dist = goal_in_robot_frame[0]


    def _reward_goal_approached2(self, goal_in_robot_frame):
        if self.last_goal_dist is not None:
            #goal_in_robot_frame : [rho, theta]
            
            # higher negative weight when moving away from goal (to avoid driving unnecessary circles when train in contin. action space)
            if (self.last_goal_dist - goal_in_robot_frame[0]) > 0:
                w = 0.3
            elif (self.last_goal_dist - goal_in_robot_frame[0]) <= 0:
                w = 0.5
            reward = round(w*(self.last_goal_dist - goal_in_robot_frame[0]), 3)

            # print("reward_goal_approached:  {}".format(reward))
            self.curr_reward += reward
        self.last_goal_dist = goal_in_robot_frame[0]


    def _reward_collision(self,laser_scan, punishment = 10):
        if laser_scan.min() <= self.robot_radius:
            self.curr_reward -= punishment
            self.info['is_done'] = True
            self.info['done_reason'] = 1
            self.info['is_success'] = 0

    
    def _reward_safe_dist(self, laser_scan, punishment = 0.15):
        if laser_scan.min() < self.safe_dist:
            self.curr_reward -= punishment


    def _reward_not_moving(self, goal_in_robot_frame, punishment = 0.01):
        # punishment for not moving
        if self.last_goal_dist == goal_in_robot_frame[0]:
            self.curr_reward -= punishment
    

    def _reward_distance_traveled(self, action = None, punishment = 0.01):
        if action is None:
            self.curr_reward -= punishment
        else:
            lin_vel = action[0]
            ang_vel = action[1]
            reward = ((lin_vel*0.97) + (ang_vel*0.03)) * 0.04
        self.curr_reward -= reward
        # print(f"reward_distance_traveled: {reward}")+

        
    def _reward_global_plan(self, global_plan, robot_pose: Pose2D, reward: float=0.1, punishment: float=0.15):
        # calculate minimal distance between robot and global path using kdtree search
        curr_dist_to_path, idx = self.get_min_dist2global_kdtree(
            global_plan, robot_pose)
        
        #if the new distance is smaller than the last one, robot gets rewarded, if it is larger, he gets penalized the larger the distance is
        if curr_dist_to_path >= self.last_dist_to_path:
            self.curr_reward += reward
        else:
            self.curr_reward -= punishment*curr_dist_to_path
        

    def get_min_dist2global_kdtree(self, global_plan, robot_pose):      
        mytree = scipy.spatial.cKDTree(global_plan)
        
        dist, index = mytree.query([robot_pose.x, robot_pose.y])
        return dist, index


