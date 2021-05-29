import numpy as np
from numpy.lib.utils import safe_eval
import rospy
import yaml
import os
import rospkg
import copy
from geometry_msgs.msg import Pose2D
from typing import Tuple
import scipy.spatial
from rl_agent.utils.debug import timeit

class RewardCalculator():
    def __init__(self, 
                 robot_radius: float, 
                 safe_dist: float, 
                 goal_radius: float, 
                 rule: str = 'rule_00' ):
        """
        A class for calculating reward based various rules.


        :param safe_dist (float): The minimum distance to obstacles or wall that robot is in safe status.
                                  if the robot get too close to them it will be punished. Unit[ m ]
        :param goal_radius (float): The minimum distance to goal that goal position is considered to be reached. 
        """
        self.curr_reward = 0
        ## TODO read it from Yaml and static obstacles
        # additional info will be stored here and be returned alonge with reward.
        self.info = {}
        self.robot_radius = robot_radius
        self.goal_radius = goal_radius
        self.last_goal_dist = None
        self.last_dist_to_path = None
        self.safe_dist = safe_dist
        #TODO: should be global setting
        self.safe_dists_human_type = self.read_saftey_distance_parameter_from_yaml()['human obstacle safety distance radius']
        self.safe_dists_robot_type = self.read_saftey_distance_parameter_from_yaml()['robot obstacle safety distance radius']
        self.safe_dists_factor = self.read_saftey_distance_parameter_from_yaml()['safety distance factor']
        self.human_obstacles_last_min = copy.deepcopy(self.safe_dists_human_type)
        for item in list(self.human_obstacles_last_min.items()):
            self.human_obstacles_last_min[item[0]] = None
        self.human_types_as_reason= copy.deepcopy(self.safe_dists_human_type)
        for i,item in enumerate(list(self.human_types_as_reason.items())):
            self.human_types_as_reason[ item[0]] = i + 3
        self.robot_obstacles_last_min = copy.deepcopy(self.safe_dists_robot_type)
        for item in list(self.robot_obstacles_last_min.items()):
            self.robot_obstacles_last_min[item[0]] = None
        self.robot_types_as_reason= copy.deepcopy(self.safe_dists_robot_type)
        for i,item in enumerate(list(self.robot_types_as_reason.items())):
            self.robot_types_as_reason[ item[0]] = i + 3 +len( self.human_types_as_reason)
        
        
        

        
        
        self.safe_dist_adult= 1.0
        self.safe_dist_child= 1.2
        self.safe_dist_elder= 1.5
        self.safe_dist_talking= 0.8
        
        self.kdtree = None
        
        self.last_goal_dist = None
        self.last_dist_to_path = None
        self.last_adult_min= None
        self.last_child_min= None
        self.last_elder_min= None
        self.cum_reward=0

        self._cal_funcs = {
            'rule_00': RewardCalculator._cal_reward_rule_00,
            'rule_01': RewardCalculator._cal_reward_rule_01,
            'rule_02': RewardCalculator._cal_reward_rule_02,
            'rule_03': RewardCalculator._cal_reward_rule_03,
            'rule_04': RewardCalculator._cal_reward_rule_04,
            'rule_05': RewardCalculator._cal_reward_rule_05,
            'rule_06': RewardCalculator._cal_reward_rule_06,
            }
        self.cal_func = self._cal_funcs[rule]

    def reset(self):
        """
        reset variables related to the episode
        """
        self.last_goal_dist = None
        self.last_dist_to_path = None
        for item in list(self.human_obstacles_last_min.items()):
           self.human_obstacles_last_min[item[0]] = None
        for item in list(self.robot_obstacles_last_min.items()):
           self.robot_obstacles_last_min[item[0]] = None
        self.cum_reward=0
  

    def _reset(self):
        """
        reset variables related to current step
        """
        self.curr_reward = 0
        self.info = {}

    def get_history_info(self):

        return list(self.human_obstacles_last_min.values())+ list(self.robot_obstacles_last_min.values())  + [self.cum_reward]
    
    def get_reward(self, 
    laser_scan:np.ndarray, 
    goal_in_robot_frame: Tuple[float,float],  
    human_obstacles_in_robot_frame:np.ndarray,
    robot_obstacles_in_robot_frame,
    robot_velocity,
    current_time_step: float, 
    *args, **kwargs):
        """
        Args:
            laser_scan (np.ndarray): 
            goal_in_robot_frame (Tuple[float,float]: position (rho, theta) of the goal in robot frame (Polar coordinate) 
            adult_in_robot_frame(np.ndarray)
        """
        self._reset()
        self.cal_func(self, laser_scan, goal_in_robot_frame, human_obstacles_in_robot_frame, robot_obstacles_in_robot_frame, robot_velocity, current_time_step,*args,**kwargs)
        self.cum_reward+=self.curr_reward
        return self.curr_reward, self.info

    def _cal_reward_rule_00(self, 
                            laser_scan: np.ndarray, 
                            goal_in_robot_frame: Tuple[float,float],
                            *args,**kwargs):
        self._reward_goal_reached(
            goal_in_robot_frame)
        self._reward_not_moving(
            kwargs['action'], punishment=0.0025)
        self._reward_safe_dist(
            laser_scan, punishment=0.25)
        self._reward_collision(
            laser_scan)
        self._reward_goal_approached(
            goal_in_robot_frame, reward_factor=0.4, penalty_factor=0.5)

    def _cal_reward_rule_01(self, 
                            laser_scan: np.ndarray, 
                            goal_in_robot_frame: Tuple[float,float],
                            *args,**kwargs):
        self._reward_distance_traveled(
            kwargs['action'], consumption_factor=0.0075)
        self._reward_goal_reached(
            goal_in_robot_frame, reward=15)
        self._reward_safe_dist(
            laser_scan, punishment=0.25)
        self._reward_collision(
            laser_scan, punishment=10)
        self._reward_goal_approached(
            goal_in_robot_frame, reward_factor=0.4, penalty_factor=0.5)

    def _cal_reward_rule_02(self, 
                            laser_scan: np.ndarray, 
                            goal_in_robot_frame: Tuple[float,float],
                            *args,**kwargs):
        self._reward_distance_traveled(
            kwargs['action'], consumption_factor=0.007)
        self._reward_following_global_plan(
            kwargs['global_plan'], kwargs['robot_pose'])
        self._reward_goal_reached(
            goal_in_robot_frame, reward=15)
        self._reward_safe_dist(
            laser_scan, punishment=0.25)
        self._reward_collision(
            laser_scan, punishment=10)
        self._reward_goal_approached(
            goal_in_robot_frame, reward_factor=0.4, penalty_factor=0.5)

    def _cal_reward_rule_03(self, laser_scan: np.ndarray, goal_in_robot_frame: Tuple[float,float], human_obstacles_in_robot_frame:np.ndarray, robot_obstacles_in_robot_frame:np.ndarray, robot_velocity:float,   current_time_step:float, *args,**kwargs):
        
        self._reward_goal_reached(goal_in_robot_frame, reward=2)
        self._reward_safe_dist(laser_scan)
        self._reward_collision(laser_scan, punishment=4)
        self._reward_goal_approached3(goal_in_robot_frame, current_time_step)
        self._reward_human_obstacles_safety_dist_all(human_obstacles_in_robot_frame,robot_velocity) #0.05 0.07
        self._reward_robot_obstacles_safety_dist_all(robot_obstacles_in_robot_frame,robot_velocity)

    def _cal_reward_rule_04(self, 
                            laser_scan: np.ndarray, 
                            goal_in_robot_frame: Tuple[float,float],
                            *args,**kwargs):
        self._reward_not_moving(
            kwargs['action'], punishment=0.0075)
        self._reward_following_global_plan(
            kwargs['global_plan'], kwargs['robot_pose'], kwargs['action'])
        if laser_scan.min() > self.safe_dist:
            self._reward_distance_global_plan(
                kwargs['global_plan'], kwargs['robot_pose'], reward_factor=0.15, penalty_factor=0.25)
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

    def _cal_reward_rule_05(self, laser_scan: np.ndarray, goal_in_robot_frame: Tuple[float,float], adult_in_robot_frame:np.ndarray, 
                                                        child_in_robot_frame:np.ndarray, elder_in_robot_frame:np.ndarray,  current_time_step:float, *args,**kwargs):
        
        self._reward_goal_reached(goal_in_robot_frame, reward=2)
        self._reward_safe_dist(laser_scan)
        self._reward_collision(laser_scan, punishment=4)
        self._reward_goal_approached3(goal_in_robot_frame,current_time_step)
        self._reward_adult_safety_dist(adult_in_robot_frame, punishment=4) #0.05 0.07
        self._reward_child_safety_dist(child_in_robot_frame, punishment=4)
        self._reward_elder_safety_dist(elder_in_robot_frame, punishment=4)

    def _cal_reward_rule_06(self, laser_scan: np.ndarray, goal_in_robot_frame: Tuple[float,float], human_obstacles_in_robot_frame:np.ndarray, robot_obstacles_in_robot_frame:np.ndarray,  current_time_step:float, *args,**kwargs):
        
        self._reward_goal_reached(goal_in_robot_frame, reward=2)
        self._reward_safe_dist(laser_scan)
        self._reward_collision(laser_scan, punishment=4)
        self._reward_goal_approached3(goal_in_robot_frame,current_time_step)
        self._reward_human_obstacles_safety_dist(human_obstacles_in_robot_frame) #0.05 0.07
        self._reward_robot_obstacles_safety_dist(robot_obstacles_in_robot_frame) #0.05 0.07

        
    def _reward_goal_reached(self,
                             goal_in_robot_frame = Tuple[float,float], 
                             reward: float=15):
        """
        Reward for reaching the goal.
        
        :param goal_in_robot_frame (Tuple[float,float]): position (rho, theta) of the goal in robot frame (Polar coordinate) 
        :param reward (float, optional): reward amount for reaching. defaults to 15
        """
        if goal_in_robot_frame[0] < self.goal_radius:
            self.curr_reward = reward
            self.info['is_done'] = True
            self.info['done_reason'] = 2
            self.info['is_success'] = 1
        else:
            self.info['is_done'] = False

    def _reward_goal_approached(self, 
                                goal_in_robot_frame = Tuple[float,float],
                                reward_factor: float=0.3,
                                penalty_factor: float=0.5):
        """
        Reward for approaching the goal.
        
        :param goal_in_robot_frame (Tuple[float,float]): position (rho, theta) of the goal in robot frame (Polar coordinate)
        :param reward_factor (float, optional): positive factor for approaching goal. defaults to 0.3
        :param penalty_factor (float, optional): negative factor for withdrawing from goal. defaults to 0.5
        """
        if self.last_goal_dist is not None:
            #goal_in_robot_frame : [rho, theta]
            
            # higher negative weight when moving away from goal 
            # (to avoid driving unnecessary circles when train in contin. action space)
            if (self.last_goal_dist - goal_in_robot_frame[0]) > 0:
                w = reward_factor
            else:
                w = penalty_factor
            reward = round(
                w*(self.last_goal_dist - goal_in_robot_frame[0]), 3)

            # print("reward_goal_approached:  {}".format(reward))
            self.curr_reward += reward
        self.last_goal_dist = goal_in_robot_frame[0]

    def _reward_adult_safety_dist(self, adult_in_robot_frame, punishment = 80):
        if adult_in_robot_frame.shape[0] != 0:
            min_adult_dist=adult_in_robot_frame[:,0].min()
            # min_adult_dist_0=adult_in_robot_frame[0,0]
            # print(min_adult_dist==min_adult_dist_0)
            if self.last_adult_min is None:
                self.last_adult_min=min_adult_dist
            else:
                if self.last_adult_min>min_adult_dist:
                    self.last_adult_min=min_adult_dist
            if min_adult_dist<self.safe_dist_adult:
                self.curr_reward -= punishment
                self.info['is_done'] = True
                self.info['done_reason'] = 3 #hit adult
                self.info['is_success'] = 0

    def _reward_child_safety_dist(self, child_in_robot_frame, punishment = 90):
        if child_in_robot_frame.shape[0]!=0:
            min_child_dist=child_in_robot_frame[:,0].min()
            if self.last_child_min is None:
                self.last_child_min=min_child_dist
            else:
                if self.last_child_min>min_child_dist:
                    self.last_child_min=min_child_dist
            if min_child_dist<self.safe_dist_child:
                self.curr_reward -= punishment
                self.info['is_done'] = True
                self.info['done_reason'] = 4 #hit child
                self.info['is_success'] = 0

    def _reward_elder_safety_dist(self, elder_in_robot_frame, punishment = 100):
        if elder_in_robot_frame.shape[0]!=0:
            min_elder_dist=elder_in_robot_frame[:,0].min()
            if self.last_elder_min is None:
                self.last_elder_min=min_elder_dist
            else:
                if self.last_elder_min>min_elder_dist:
                    self.last_elder_min=min_elder_dist
            if min_elder_dist<self.safe_dist_elder:
                self.curr_reward -= punishment
                self.info['is_done'] = True
                self.info['done_reason'] = 5 # hit elder
                self.info['is_success'] = 0

    def  _reward_human_obstacles_safety_dist(self, human_obstacles_in_robot_frame):
        ### split the array into multiple array depending on type 

        human_obstacles_in_robot_frame = [human_obstacles_in_robot_frame[human_obstacles_in_robot_frame[:,3]==k] for k in np.unique(human_obstacles_in_robot_frame[:,3])]
      
        for human_obstacles_by_type_in_robot_frame in human_obstacles_in_robot_frame :

            if human_obstacles_by_type_in_robot_frame.shape[0]!=0:
                type = human_obstacles_by_type_in_robot_frame[0,2]
                behavior = human_obstacles_by_type_in_robot_frame[0,1]
                
                min_human_obstacle_dist=human_obstacles_by_type_in_robot_frame[0,0]
                min_human_obstacle_behavior=human_obstacles_by_type_in_robot_frame[0,1]
                if self.human_obstacles_last_min[type] is None:
                    self.human_obstacles_last_min[type]=min_human_obstacle_dist
                else:
                    if self.human_obstacles_last_min[type]>min_human_obstacle_dist:
                        self.human_obstacles_last_min[type]=min_human_obstacle_dist
               
                safe_dist_=self.safe_dists_human_type[type] * self.safe_dists_factor[behavior]
                punishment = 0.07 * safe_dist_
                if min_human_obstacle_dist<safe_dist_:
                    self.curr_reward -= punishment
                    self.info['is_done'] = True
                    self.info['done_reason'] = self.human_types_as_reason[type] #hit
                    self.info['is_success'] = 0
    
    def  _reward_robot_obstacles_safety_dist(self, robot_obstacles_in_robot_frame):
        ### split the array into multiple array depending on type 

        robot_obstacles_in_robot_frame = [robot_obstacles_in_robot_frame[robot_obstacles_in_robot_frame[:,2]==k] for k in np.unique(robot_obstacles_in_robot_frame[:,2])]
      
        for robot_obstacles_by_type_in_robot_frame in robot_obstacles_in_robot_frame :

            if robot_obstacles_by_type_in_robot_frame.shape[0]!=0:
                type = robot_obstacles_by_type_in_robot_frame[0,1]
              
                min_robot_obstacle_dist=robot_obstacles_by_type_in_robot_frame[0,0]

                if self.robot_obstacles_last_min[type] is None:
                    self.robot_obstacles_last_min[type]=min_robot_obstacle_dist
                else:
                    if self.robot_obstacles_last_min[type]>min_robot_obstacle_dist:
                        self.robot_obstacles_last_min[type]=min_robot_obstacle_dist
               
                safe_dist_=self.safe_dists_robot_type[type] 
                punishment = 0.07 * safe_dist_
                if min_robot_obstacle_dist<safe_dist_:
                    self.curr_reward -= punishment
                    self.info['is_done'] = True
                    self.info['done_reason'] = self.robot_types_as_reason[type] #hit
                    self.info['is_success'] = 0

    def  _reward_human_obstacles_safety_dist_all(self, human_obstacles_in_robot_frame, robot_velocity):
        ### split the array into multiple array depending on type 

        human_obstacles_in_robot_frame = [human_obstacles_in_robot_frame[human_obstacles_in_robot_frame[:,3]==k] for k in np.unique(human_obstacles_in_robot_frame[:,3])]
      
        for human_obstacles_by_type_in_robot_frame in human_obstacles_in_robot_frame :

            if human_obstacles_by_type_in_robot_frame.shape[0]!=0:
                type = human_obstacles_by_type_in_robot_frame[0,2]
                behavior = human_obstacles_by_type_in_robot_frame[0,1]
                
                min_human_obstacle_dist=human_obstacles_by_type_in_robot_frame[0,0]
                min_human_obstacle_behavior=human_obstacles_by_type_in_robot_frame[0,1]
                if self.human_obstacles_last_min[type] is None:
                    self.human_obstacles_last_min[type]=min_human_obstacle_dist
                else:
                    if self.human_obstacles_last_min[type]>min_human_obstacle_dist:
                        self.human_obstacles_last_min[type]=min_human_obstacle_dist
                for dist in human_obstacles_by_type_in_robot_frame:
                    safe_dist_=self.safe_dists_human_type[type] * self.safe_dists_factor[behavior]
                    punishment = 0.07 * safe_dist_
                    if dist[0]<safe_dist_:
                        self.curr_reward -= punishment*np.exp(1-dist[0]/safe_dist_) + robot_velocity * 0.2

            
    def  _reward_robot_obstacles_safety_dist_all(self, robot_obstacles_in_robot_frame, robot_velocity):
        ### split the array into multiple array depending on type 

        robot_obstacles_in_robot_frame = [robot_obstacles_in_robot_frame[robot_obstacles_in_robot_frame[:,2]==k] for k in np.unique(robot_obstacles_in_robot_frame[:,2])]
      
        for robot_obstacles_by_type_in_robot_frame in robot_obstacles_in_robot_frame :

            if robot_obstacles_by_type_in_robot_frame.shape[0]!=0:
                type = robot_obstacles_by_type_in_robot_frame[0,1]
              
                min_robot_obstacle_dist=robot_obstacles_by_type_in_robot_frame[0,0]

                if self.robot_obstacles_last_min[type] is None:
                    self.robot_obstacles_last_min[type]=min_robot_obstacle_dist
                else:
                    if self.robot_obstacles_last_min[type]>min_robot_obstacle_dist:
                        self.robot_obstacles_last_min[type]=min_robot_obstacle_dist
                for dist in robot_obstacles_by_type_in_robot_frame:
                    safe_dist_=self.safe_dists_robot_type[type] 
                    punishment = 0.07* safe_dist_
                    if dist[0]<safe_dist_:
                        self.curr_reward -= punishment*np.exp(1-dist[0]/safe_dist_) + robot_velocity * 0.2

                
   



    def _reward_goal_approached3(self, goal_in_robot_frame, current_time_step):
        if self.last_goal_dist is not None:
            # higher negative weight when moving away from goal (to avoid driving unnecessary circles when train in contin. action space)
            if (self.last_goal_dist - goal_in_robot_frame[0]) > 0:
                w = 0.018*np.exp(1-current_time_step)
            elif (self.last_goal_dist - goal_in_robot_frame[0]) < 0:
                w = -0.05*np.exp(1)
            else:
                w = -0.03
            reward = round(w, 5)

            # print("reward_goal_approached:  {}".format(reward))
            self.curr_reward += reward
        self.last_goal_dist = goal_in_robot_frame[0]

    def _reward_adult_safety_dist3(self, adult_in_robot_frame, punishment = 80):
        if adult_in_robot_frame.shape[0] != 0:
            min_adult_dist=adult_in_robot_frame.min()
            if self.last_adult_min is None:
                self.last_adult_min=min_adult_dist
            else:
                if self.last_adult_min>min_adult_dist:
                    self.last_adult_min=min_adult_dist
            for dist in adult_in_robot_frame:
                if dist<self.safe_dist_adult:
                    self.curr_reward -= punishment*np.exp(1-dist/self.safe_dist_adult)


    def _reward_child_safety_dist3(self, child_in_robot_frame, punishment = 90):
        if child_in_robot_frame.shape[0]!=0:
            min_child_dist=child_in_robot_frame.min()
            if self.last_child_min is None:
                self.last_child_min=min_child_dist
            else:
                if self.last_child_min>min_child_dist:
                    self.last_child_min=min_child_dist
            for dist in child_in_robot_frame:
                if dist<self.safe_dist_child:
                    self.curr_reward -= punishment*np.exp(1-dist/self.safe_dist_adult)


    def _reward_elder_safety_dist3(self, elder_in_robot_frame, punishment = 100):
        if elder_in_robot_frame.shape[0]!=0:
            min_elder_dist=elder_in_robot_frame.min()
            if self.last_elder_min is None:
                self.last_elder_min=min_elder_dist
            else:
                if self.last_elder_min>min_elder_dist:
                    self.last_elder_min=min_elder_dist
            for dist in elder_in_robot_frame:
                if dist<self.safe_dist_elder:
                    self.curr_reward -= punishment*np.exp(1-dist/self.safe_dist_adult)

    def _reward_collision(self,
                          laser_scan: np.ndarray, 
                          punishment: float=10):
        """
        Reward for colliding with an obstacle.
        
        :param laser_scan (np.ndarray): laser scan data
        :param punishment (float, optional): punishment for collision. defaults to 10
        """
        if laser_scan.min() <= self.robot_radius:
            self.curr_reward -= punishment
            self.info['is_done'] = True
            self.info['done_reason'] = 1
            self.info['is_success'] = 0

    def _reward_safe_dist(self, 
                          laser_scan: np.ndarray, 
                          punishment: float=0.15):
        """
        Reward for undercutting safe distance.
        
        :param laser_scan (np.ndarray): laser scan data
        :param punishment (float, optional): punishment for undercutting. defaults to 0.15
        """
        if laser_scan.min() < self.safe_dist:
            self.curr_reward -= punishment

    def _reward_not_moving(self, 
                           action: np.ndarray=None, 
                           punishment: float=0.01):
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
                self.curr_reward -= punishment/2

    def _reward_distance_traveled(self, 
                                  action: np.array = None, 
                                  punishment: float=0.01,
                                  consumption_factor: float=0.005):
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
            reward = (lin_vel + (ang_vel*0.001)) * consumption_factor
        self.curr_reward -= reward
        
    def _reward_distance_global_plan(self, 
                                     global_plan: np.array, 
                                     robot_pose: Pose2D,
                                     reward_factor: float=0.1, 
                                     penalty_factor: float=0.15):
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
                                      dist_to_path: float=0.5):
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


    def read_saftey_distance_parameter_from_yaml(self):
        
        file_location = os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'saftey_distance_parameter.yaml')
        
        
        if os.path.isfile(file_location):
            with open(file_location, "r") as file:
                saftey_distance_parameter = yaml.load(file, Loader=yaml.FullLoader)       
        assert isinstance(
             saftey_distance_parameter, dict), "'saftey_distance_parameter.yaml' has wrong fromat! Has to encode dictionary!"
                
        return saftey_distance_parameter
        
