import numpy as np
import math
from numpy.lib.utils import safe_eval
import rospy
from geometry_msgs.msg import Pose2D, PoseStamped
from typing import Tuple
import scipy.spatial
from rl_agent.utils.debug import timeit

class RewardCalculator():
    def __init__(self, 
                 robot_radius: float, 
                 safe_dist: float, 
                 goal_radius: float, 
                 rule: str = 'rule_00',
                 extended_eval: bool = False):
        """
        A class for calculating reward based various rules.


        :param safe_dist (float): The minimum distance to obstacles or wall that robot is in safe status.
                                  if the robot get too close to them it will be punished. Unit[ m ]
        :param goal_radius (float): The minimum distance to goal that goal position is considered to be reached. 
        """
        self.curr_reward = 0
        # additional info will be stored here and be returned alonge with reward.
        self.info = {}
        self.robot_radius = robot_radius
        self.goal_radius = goal_radius
        self.last_goal_dist = None
        self.last_dist_to_path = None
        self.last_action = None
        self.safe_dist = safe_dist
        self._extended_eval = extended_eval

        self.kdtree = None

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
        self.last_action = None
        self.kdtree = None

    def _reset(self):
        """
        reset variables related to current step
        """
        self.curr_reward = 0
        self.info = {}
    
    def get_reward(self, 
                   laser_scan: np.ndarray, 
                   goal_in_robot_frame: Tuple[float,float], 
                   *args, **kwargs):
        """
        Returns reward and info to the gym environment.

        :param laser_scan (np.ndarray): laser scan data
        :param goal_in_robot_frame (Tuple[float,float]): position (rho, theta) of the goal in robot frame (Polar coordinate)  
        """
        self._reset()
        self.cal_func(self,laser_scan,goal_in_robot_frame,*args,**kwargs)
        return self.curr_reward, self.info

    def _cal_reward_rule_00(self, 
                            laser_scan: np.ndarray, 
                            goal_in_robot_frame: Tuple[float,float],
                            *args,**kwargs):
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
            goal_in_robot_frame, reward_factor=0.3, penalty_factor=0.4)

    def _cal_reward_rule_02(self, 
                            laser_scan: np.ndarray, 
                            goal_in_robot_frame: Tuple[float,float],
                            *args,**kwargs):
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
                            goal_in_robot_frame: Tuple[float,float],
                            *args,**kwargs):
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
                             goal_in_robot_frame: Tuple[float,float],
                             *args,**kwargs):
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

    def _cal_reward_rule_05(self, 
                             laser_scan: np.ndarray, 
                             goal_in_robot_frame: Tuple[float,float],
                             *args,**kwargs):
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
        self._reward_time_elapsed(kwargs['time_elapsed'], kwargs['global_plan'])

    def _cal_reward_rule_06(self, 
                             laser_scan: np.ndarray, 
                             goal_in_robot_frame: Tuple[float,float],
                             *args,**kwargs):
        self._rule = 6                     
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
        #self._reward_time_elapsed(kwargs['time_elapsed'], kwargs['global_plan'])
        self._reward_waypoints_set_near_global_plan(kwargs['goal'], kwargs['subgoal'], kwargs['last_subgoal'],kwargs['amount_rewarded_subgoals'], kwargs['global_plan'], kwargs['robot_close_to_goal'], self._rule)

    def _cal_reward_rule_07(self, 
                             laser_scan: np.ndarray, 
                             goal_in_robot_frame: Tuple[float,float],
                             *args,**kwargs):
        self._rule = 7                   
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
        #self._reward_time_elapsed(kwargs['time_elapsed'], kwargs['global_plan'])
        self._reward_waypoints_set_near_global_plan(kwargs['goal'], kwargs['subgoal'], kwargs['last_subgoal'],kwargs['amount_rewarded_subgoals'], kwargs['global_plan'], kwargs['robot_close_to_goal'], self._rule)

    def _cal_reward_rule_08(self, 
                             laser_scan: np.ndarray, 
                             goal_in_robot_frame: Tuple[float,float],
                             *args,**kwargs):
        self._rule = 8                   
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
        #self._reward_time_elapsed(kwargs['time_elapsed'], kwargs['global_plan'])
        self._reward_waypoints_set_near_global_plan(kwargs['goal'], kwargs['subgoal'], kwargs['last_subgoal'],kwargs['amount_rewarded_subgoals'], kwargs['global_plan'], kwargs['robot_close_to_goal'], self._rule)

    def _reward_waypoints_set_near_global_plan(self,
                             goal: Pose2D,
                             subgoal: PoseStamped,
                             last_subgoal: PoseStamped,
                             amount_rewarded_subgoals: int,
                             global_plan: np.array,
                             robot_close_to_goal: bool, 
                             rule: int):
        """
        Reward for putting a reward close to the global path.
        The further away from the global plan a waypoint is set, the less points are archived by this reward.
        There will be no reward for placing it onto the goal, because this is not learned but coded to be done, when the robot gets close enough.
        The reward will be devided by estimated amounts of waypoints set, which is 1 waypoint each 100 meter (of global plan)
        
        :param goal (Pose2D): the global goal, which is to be reached
        :param subgoal (PoseStamped): the current subgoal, which is to be reached. Gains a reward, if different from last_subgoal
        :param last_subgoal (PoseStamped): the last subgoal, which was rewarded. Used to check, if a new subgoal has been published, which needs to be rewarded
        :param amount_rewarded_subgoal (int): amount of already rewarded subgoals. Used for punishment, if too many subgoals are used
        :param global_plan: (np.ndarray): vector containing poses on global plan
        :param robot_close_to_goal (bool): Boolean to signalize that robot is close to goal
        :param rule: (int): The rule from which the method is called
        """

        scaling_factor = 1
        estimatedSubgoalAmount = math.floor(len(global_plan)/100) -1
        if rule in [6]:
            if estimatedSubgoalAmount == -1:
                estimatedSubgoalAmount = 0
            scaling_factor = 1 / (estimatedSubgoalAmount +1)
            estimatedSubgoalAmount = 100000
        max_subgoal_gp_dist = 1.5 # range of the circle around ref-subgoal set in wp3_env
        #give reward if subgoal is not the same as last step (except goal)
        self.info['subgoal_was_rewarded'] = True
        if not (subgoal.pose.position.x == last_subgoal.pose.position.x and subgoal.pose.position.y == last_subgoal.pose.position.y) or (subgoal.pose.position.x == goal.x and subgoal.pose.position.y == goal.y):
            if amount_rewarded_subgoals >= estimatedSubgoalAmount:
                #punishment, if too many waypoints were set. Don't punish if waypoint is on the goal
                if not (subgoal.pose.position.x == goal.x and subgoal.pose.position.y == goal.y):
                    self.curr_reward -= 0.5 * scaling_factor
            else:
                if subgoal.pose.position.x == goal.x and subgoal.pose.position.y == goal.y:
                    if robot_close_to_goal:
                        self.curr_reward += 1 * scaling_factor
                    else:
                        self.curr_reward -= 5 * scaling_factor
                        #print("set subgoal on goal too early!")
                else:
                    subgoal_gp_dist = np.inf
                    for gp_point in global_plan:
                        dist = ((gp_point[0] - subgoal.pose.position.x)**2 + (gp_point[1] - subgoal.pose.position.y)**2)**0.5
                        if dist < subgoal_gp_dist:
                            subgoal_gp_dist = dist
                    if not np.isposinf(subgoal_gp_dist):
                        if rule in [6,8]:
                            self.curr_reward += (1 - ((subgoal_gp_dist/max_subgoal_gp_dist) ** 0.5)) * scaling_factor
                        else:
                            self.curr_reward += 1 * scaling_factor
                    else:
                        self.info['subgoal_was_rewarded'] = False
        else:
            self.info['subgoal_was_rewarded'] = False

    def _reward_time_elapsed(self,
                             time_elapsed: float,
                             global_plan: np.array):
        """
        Reward for being fast / punishment for being slow.
        If the run is in range of 70% of the best time/ meter of global path there will be a reaward. 
        When the 70% mark is exceeded, there will be a punishment and the run will fail with reason 'max steps exceeded'
        
        :param time_elapsed (float): time, which elapsed since the first action was published 
        :param global_plan: (np.ndarray): vector containing poses on global plan
        """
        time_factor = 0.058180871140939415 #time / meter of global path needed for the best 70% of evaluated runs
        if time_factor * len(global_plan) - time_elapsed >= 0.5* time_factor * len(global_plan): #if more than 50% of the available time is left, then get a higher reward, to not reward slow robots inadequate.
            self.curr_reward += 0.019
        elif time_factor * len(global_plan) - time_elapsed <= 0: # if available time is exhausted, punish the robot instead
            self.curr_reward -= 0.01
            if self.info['is_done'] == True: #if goal was reached, but the robot did not satisfy the target time, the 15 points granted by reaching the goal need to be removed.
                self.curr_reward -= 15 
            self.info['done_reason'] = 0
            self.info['is_success'] = 0 
        else: #in this case the robot is in the lower half of the available time, now he only gets small rewards, so it is not worth to slow down to accumulate rewards, while available time is not exhausted.
            self.curr_reward += 0.001

    def _reward_goal_reached(self,
                             goal_in_robot_frame = Tuple[float,float], 
                             reward: float=15):
        """
        Reward for reaching the goal.
        
        :param goal_in_robot_frame (Tuple[float,float]): position (rho, theta) of the goal in robot frame (Polar coordinate) 
        :param reward (float, optional): reward amount for reaching. defaults to 15
        """
        if goal_in_robot_frame[0] < self.goal_radius:
            self.curr_reward += reward
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
            reward = w*(self.last_goal_dist - goal_in_robot_frame[0])

            # print("reward_goal_approached:  {}".format(reward))
            self.curr_reward += reward
        self.last_goal_dist = goal_in_robot_frame[0]

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
            
            if not self._extended_eval:
                self.info['is_done'] = True
                self.info['done_reason'] = 1
                self.info['is_success'] = 0
            else:
                self.info['crash'] = True

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
            
            if self._extended_eval:
                self.info['safe_dist'] = True

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

    def _reward_abrupt_direction_change(self,
                                        action: np.array=None):
        """
        Applies a penalty when an abrupt change of direction occured.
        
        :param action: (np.ndarray (,2)): [0] = linear velocity, [1] = angular velocity 
        """
        if self.last_action is not None:
            curr_ang_vel = action[1]
            last_ang_vel = self.last_action[1]

            vel_diff = abs(curr_ang_vel-last_ang_vel)
            self.curr_reward -= (vel_diff**4)/2500
        self.last_action = action
