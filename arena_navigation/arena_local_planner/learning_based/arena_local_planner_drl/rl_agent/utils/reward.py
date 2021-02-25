import numpy as np
from numpy.lib.utils import safe_eval
import rospy
from typing import Tuple
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Path
import scipy
import math
 


class RewardCalculator():
    def __init__(self, robot_radius: float, safe_dist:float, goal_radius:float, rule:str = 'rule_00'):
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
        self.last_dist2global = 0
        self.safe_dist = safe_dist
        self._cal_funcs = {
            'rule_00': RewardCalculator._cal_reward_rule_00,
            'rule_01': RewardCalculator._cal_reward_rule_01,
            'rule_02': RewardCalculator._cal_reward_rule_02
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
    
    def get_reward(self, laser_scan:np.ndarray, 
                    goal_in_robot_frame: Tuple[float,float],
                    robot_position: Pose2D(),
                    globalPlan: Path(), *args, **kwargs):
        """

        Args:
            laser_scan (np.ndarray): 
            goal_in_robot_frame (Tuple[float,float]: position (rho, theta) of the goal in robot frame (Polar coordinate)  
            robot_position 
        """

        self._reset()
        self.cal_func(self,laser_scan,goal_in_robot_frame, robot_position, globalPlan, *args,**kwargs)
        return self.curr_reward, self.info


    def _cal_reward_rule_00(self, laser_scan: np.ndarray, goal_in_robot_frame: Tuple[float,float],robot_position:Pose2D(), globalPlan: Path(),*args,**kwargs):
        self._reward_distance_traveled(kwargs['action'])
        self._reward_waypoints_set(kwargs['goal_len'], kwargs['action_count'])
        self._reward_goal_reached(goal_in_robot_frame)
        self._reward_not_moving(goal_in_robot_frame)
        self._reward_safe_dist(laser_scan)
        self._reward_collision(laser_scan)
        self._reward_goal_approached(goal_in_robot_frame)
        #self._reward_waypoint_on_global(globalPlan, kwargs['subgoal'])
        

    def _cal_reward_rule_01(self, laser_scan: np.ndarray, goal_in_robot_frame: Tuple[float,float],*args,**kwargs):
        self._reward_distance_traveled(kwargs['action'])
        self._reward_waypoints_set(kwargs['goal_len'], kwargs['action_count'])
        self._reward_goal_reached(goal_in_robot_frame)
        self._reward_not_moving(goal_in_robot_frame)
        self._reward_safe_dist(laser_scan)
        self._reward_collision(laser_scan)
        self._reward_goal_approached2(goal_in_robot_frame)


    def _cal_reward_rule_02(self, laser_scan: np.ndarray, goal_in_robot_frame: Tuple[float,float], robot_position: Pose2D(), globalPlan: Path(), *args,**kwargs):
        self._reward_distance_traveled(kwargs['action'])
        self._reward_waypoints_set(kwargs['goal_len'], kwargs['action_count'])
        self._reward_goal_reached(goal_in_robot_frame)
        self._reward_not_moving(goal_in_robot_frame)
        self._reward_safe_dist(laser_scan)
        self._reward_collision(laser_scan)
        self._reward_goal_approached(goal_in_robot_frame)
        self._reward_global_plan_followed(goal_in_robot_frame, robot_position, globalPlan.poses)
        

    def _reward_goal_reached(self,goal_in_robot_frame, reward = 15, punishment = 15):
        print("goal radius: {}".format(self.goal_radius))
        if goal_in_robot_frame[0] < self.goal_radius:
            self.curr_reward = reward
            self.info['is_done'] = True
            self.info['done_reason'] = 2
        # if goal_in_robot_frame[0] > 10:
        #     self.curr_reward -= punishment
        #     self.info['is_done'] = True
        #     self.info['done_reason'] = 1
        else:
            self.info['is_done'] = False



    def _reward_goal_approached(self, goal_in_robot_frame,reward = 1, punishment = 0.0001):
        if self.last_goal_dist is not None:
            # goal_in_robot_frame : [rho, theta]
            # if current goal distance shorter than last one, positive weighted reward - otherwise negative wegihted reward
            w = 0.25
            reward = round(w*(self.last_goal_dist - goal_in_robot_frame[0]), 3)
            
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
            self.info['is_success'] = 0

    
    def _reward_safe_dist(self, laser_scan, punishment = 0.15):
        if laser_scan.min() < self.safe_dist:
            self.curr_reward -= punishment

    def _reward_global_plan_followed(self, goal_in_robot_frame, robot_position, globalPlan,  reward = 0.3, punishment = 0.0001):
        """
            make sure robots keeps following global path ensure distance to global path is low
        """
        #calculate minimal distance between robot and global path using kdtree search
        new_dist2global, dist2global_index = self.get_min_dist2global_kdtree(globalPlan, robot_position)
        print("newdist2globalpath: {}".format(new_dist2global))
        #if the new distance is smaller than the last one, robot gets rewarded, if it is larger, he gets penalized the larger the distance is    
        if new_dist2global < self.last_dist2global:
            reward += reward
            self.last_dist2global = new_dist2global
            print("reward because nearer to global path")
        else:
            reward -= punishment*new_dist2global
            print("punish because to far from global path")
    
    def get_min_dist2global_kdtree(globalPlan ,robot_position):

        mytree = scipy.spatial.cKDTree(globalPlan.pose)
        dist, index = mytree.query(robot_position)
        return dist, index

    def _reward_not_moving(self, goal_in_robot_frame, punishment = 0.01):
        # punishment for not moving
        if self.last_goal_dist == goal_in_robot_frame[0]:
            self.curr_reward -= punishment
    

    def _reward_distance_traveled(self, action = None, punishment = 1):
        if action is None:
            self.curr_reward -= punishment
        else:
            lin_vel = action[0]
            ang_vel = action[1]
            reward = ((lin_vel*0.97) + (ang_vel*0.03)) * 0.04
        self.curr_reward -= reward
        # print(f"reward_distance_traveled: {reward}")
        # print(" linear velocity in distance reward fct {}".format(lin_vel))
        # print(" angular velocity in distance reward fct {}".format(ang_vel))

    def _reward_waypoints_set(self, goal_len, action_count, punishment = -0.1):
        if action_count > goal_len:
            self.curr_reward -= punishment

    def _reward_waypoint_on_global(self, globalPlan, subgoal, reward = 1):
        # if waypoint is on global path then give additional reward
        # ! code not efficient since it searches the whole global path array, needs optimization
        i = 0
        while i < len(globalPlan.poses)/100:    
            if (math.isclose(subgoal.x, globalPlan.poses[i].pose.position.x, rel_tol=0.4) and math.isclose(subgoal.y, globalPlan.poses[i].pose.position.y, rel_tol=0.4)): 
                self.curr_reward += reward
            i += 1
                
            
