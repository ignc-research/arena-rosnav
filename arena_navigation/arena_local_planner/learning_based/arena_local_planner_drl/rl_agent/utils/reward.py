import enum
from os import sendfile
from pickle import TRUE
from re import S
import numpy as np
from numpy.lib.utils import safe_eval
from torch._C import set_flush_denormal
from rl_agent.config.config import configurable
import rospy
from geometry_msgs.msg import Pose2D
from typing import Tuple,Optional, overload
import scipy.spatial
from rl_agent.utils.debug import timeit
from rl_agent.utils.observation_collector import ObservationCollectorWP2,ObservationCollectorWP
import weakref

from rospy.rostime import is_rostime_initialized

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

# dont want to change the file's structure

from ..utils import Registry
from ..config import CfgNode
from ..feature_extract import get_policy_kwargs

REWARD_REGISTRY = Registry('reward calculator registry')

def build_reward_calculator(cfg:CfgNode,observation_collector,wp_env):
    reward_calculator_name = cfg.REWARD.RULE_NAME
    return REWARD_REGISTRY.get(reward_calculator_name)(cfg,observation_collector,wp_env)

@REWARD_REGISTRY.register
class RewardCalculatorWP_RULE00():
    INFO_KEYS = (
        'num_times_not_safe',
        'traj_dist_ratio_min',
        "traj_dist_ratio_max",
        'traj_dist_ratio_median',
        'traj_theta_std_min',
        'traj_theta_std_max',
        'traj_theta_std_median')
    @configurable
    def __init__(self,
                 observation_collector:ObservationCollectorWP,
                 wp_env: "WPEnv",
                 safe_dist:float,
                 safe_dist_reward_factor:float,
                 running_timeout_factor=1.1,
                 collision_reward_factor = 1.1,
                 traj_reward_factor = 0.2,
                 traj_dist_ratio_thresh:Tuple[float,float]=[1.3,1.7],
                 traj_theta_std_thresh = 0.5,
                 global_goal_reached_reward = 16,
                 step_base_reward: Optional[float] = 1,
                 ):
        """
        A class for calculating reward based various rules for waypoint generator.


        :param safe_dist (float): The minimum distance to obstacles or wall that robot is in safe status.
                                  if the robot get too close to them it will be punished. Unit[ m ]
        :param goal_radius (float): The minimum distance to goal that goal position is considered to be reached. 
        """
        self.curr_reward = 0
        self.oc = observation_collector
        self.wp_env  = weakref.proxy(wp_env)


        # factors
        self.running_timeout_factor = running_timeout_factor
        self.collision_reward_factor = collision_reward_factor
        # this factor will be used for variable start with traj
        self.traj_reward_factor = traj_reward_factor
        self.traj_dist_ratio_thresh = traj_dist_ratio_thresh
        # use this to check spin
        self.traj_theta_std_thresh = traj_theta_std_thresh
        self.global_goal_reached_reward = global_goal_reached_reward
        self.step_base_reward  = step_base_reward
        self.safe_dist = safe_dist
        self.safe_dist_reward_factor = safe_dist_reward_factor


    @classmethod
    def from_config(cls,cfg:CfgNode,observation_collector,wp_env):
        return dict(
            observation_collector=observation_collector,
            wp_env = wp_env,
            safe_dist = cfg.REWARD.SAFE_DIST,
            safe_dist_reward_factor = cfg.REWARD.SAFE_DIST_REWARD_FACTOR,
            running_timeout_factor=cfg.REWARD.RUNNING_TIMEOUT_FACTOR,
            collision_reward_factor = cfg.REWARD.COLLISION_REWARD_FACTOR,
            traj_reward_factor = cfg.REWARD.TRAJ_REWARD_FACTOR,
            traj_dist_ratio_thresh = cfg.REWARD.TRAJ_DIST_RATIO_THRESH,
            traj_theta_std_thresh = cfg.REWARD.TRAJ_THETA_STD_THRESH,
            global_goal_reached_reward = cfg.REWARD.GLOBAL_GOAL_REACHED_REWARD,
            step_base_reward = cfg.REWARD.STEP_BASE_REWARD,
        )



    def _reset(self):
        """
        reset variables related to current step
        """
        self.curr_reward = 0

    def reset_on_episode_start(self):
        self.num_times_not_safe = 0
        self.traj_dist_ratios=[]
        self.traj_theta_stds = []
        self.info = {
            'num_times_not_safe':0,
            'traj_dist_ratio_min':-1,
            'traj_dist_ratio_max':-1,
            'traj_dist_ratio_median':-1,
            'traj_theta_std_min':-1,
            'traj_theta_std_max': -1,
            'traj_theta_std_median':-1,
        }
    def get_reward_info(self):
        return self.info

    def save_info_on_episode_end(self):
        self.info['num_times_not_safe']= self.num_times_not_safe
        traj_dist_ratios = np.array(self.traj_dist_ratios)
        if len(traj_dist_ratios):
            self.info['traj_dist_ratio_min'] = round(traj_dist_ratios.min(),4)
            self.info['traj_dist_ratio_max'] = round(traj_dist_ratios.max(),4)
            self.info['traj_dist_ratio_median'] = round(np.median(traj_dist_ratios),4)

        traj_theta_stds = np.array(self.traj_theta_stds)
        if len(traj_theta_stds):
            self.info['traj_theta_std_min'] = round(traj_theta_stds.min(),4)
            self.info['traj_theta_std_max'] = round(traj_theta_stds.max(),4)
            self.info['traj_theta_std_median'] = round(np.median(traj_theta_stds),4)
    
    def get_reward_global_goal_reached(self):
        self._reset()
        return self.global_goal_reached_reward

    def cal_reward(self):
        """
        Returns reward and info to the gym environment.
        """
        self._reset()
        self._set_curr_base_reward()
        if self.oc.important_event == ObservationCollectorWP.Event.TIMEOUT:
            self._reward_timeout()
        elif self.oc.important_event == ObservationCollectorWP.Event.COLLISIONDETECTED:
            self._reward_collision()
        else:
            self._reward_actual_traj()
        return self.curr_reward

    def _set_curr_base_reward(self):

        self.curr_base_reward = self.step_base_reward
       

    def _reward_collision(self,skip = 0):
        """check collision
        """
        self.curr_reward -= self.curr_base_reward*self.collision_reward_factor

    def _reward_safe_dist(self,skip = 0):
        """

        Args:
            skip (int, optional): [description]. Defaults to 0.
        """
        laser_scans = []
        for i, scan in enumerate(self.oc._laser_scans):
            if i%(skip+1) == 0:
                laser_scans.append(scan)
        laser_scans = np.array(laser_scans)
        num_times_not_safe  = np.any(laser_scans<self.safe_dist,axis=0).sum()
        self.num_times_not_safe += num_times_not_safe
        self.curr_reward -= self.curr_base_reward*self.safe_dist_reward_factor*num_times_not_safe/len(laser_scans)
    

    def _reward_actual_traj(self):
        """check spin or something happend
        """
        robot_xytheta = np.array(list(map(lambda robot_state: [robot_state[0].x,robot_state[0].y,robot_state[0].theta],self.oc._robot_states)))
        dist_traj = np.linalg.norm(robot_xytheta[1:,:-1]-robot_xytheta[:-1,:-1],axis=1).sum()
        waypoint_x = self.wp_env._waypoint_x
        waypoint_y = self.wp_env._waypoint_y
        init_robot_pos = self.oc._robot_states[0][0]
        dist_waypoint_robot = ((init_robot_pos.x-waypoint_x)**2+(init_robot_pos.y-waypoint_y)**2)**0.5
        traj_dist_ratio = dist_traj/(dist_waypoint_robot-self.wp_env._robot_waypoint_min_dist)
        self.traj_dist_ratios.append(traj_dist_ratio)

        # distance trajectory
        if traj_dist_ratio>self.traj_dist_ratio_thresh[1]:
            self.curr_reward -= self.curr_base_reward*self.traj_reward_factor
        elif traj_dist_ratio<self.traj_dist_ratio_thresh[0]:
            self.curr_reward += self.curr_base_reward*self.traj_reward_factor

        # check robot spin
        thetas = robot_xytheta[:,2]
        # use inita theta as reference 
        thetas = robot_xytheta[:,2]-robot_xytheta[0,2]
        std_theta = np.std(thetas)
        self.traj_theta_stds.append(std_theta)
        if std_theta > self.traj_theta_std_thresh:
            self.curr_reward -= self.curr_base_reward*self.traj_reward_factor
        

    def _reward_timeout(self)-> bool:
        """set reward for timeout

        Returns:
            is_timeout(bool)
        """
        self.curr_reward -= self.curr_base_reward*self.running_timeout_factor

    
@REWARD_REGISTRY.register
class RewardCalculatorWP_RULE01():
    """ use a non-fixed step base reward
    """
    @overload
    def _set_curr_base_reward(self):
        """because the waypoint is set in or on the circle centered at the subgoal,if we don't adaptively change
            the base reward, the network will always pefer to set a waypoint closed to the robot,which takes more steps 
            and get more accumulative reward.
        """
        self.curr_base_reward = self.step_base_reward
        subgoal = self.oc.get_subgoal_pos()
        subgoal_x = subgoal.x
        subgoal_y =  subgoal.y
        waypoint_x = self.wp_env._waypoint_x
        waypoint_y = self.wp_env._waypoint_y
        old_subgoal =self.oc._old_subgoal
        if old_subgoal is None:
            init_robot_pos = self.oc._robot_states[0][0]        
            dist_waypoint_robot = ((init_robot_pos.x-waypoint_x)**2+(init_robot_pos.y-waypoint_y)**2)**0.5
            dist_subgoal_robot = ((init_robot_pos.x-subgoal_x)**2+(init_robot_pos.y-subgoal_y)**2)**0.5
            self.curr_base_reward *=  dist_waypoint_robot/dist_subgoal_robot
        else:
            dist_waypoint_old_subgoal = ((old_subgoal.x-waypoint_x)**2+(old_subgoal.y-waypoint_y)**2)**0.5
            dist_subgoals = ((old_subgoal.x-subgoal_x)**2+(old_subgoal.y-subgoal_y)**2)**0.5
            self.curr_base_reward *=  dist_waypoint_old_subgoal/dist_subgoals


    

@REWARD_REGISTRY.register
class RewardCalculatorWP_TUAHN00():
    """In Tuah's reward settings,follwing reward's calculating rules have been defined.
        1. reward_goal_reached
        2. reward_goal_approached(rejected, no sense for waypoint generator)

        3. reward_safe_dist
        4. reward_collision
        
        # 5. reward_following_global_plan(a good way to implictly encode the cost )
    """
    @configurable
    def __init__(self,
                 observation_collector:ObservationCollectorWP,
                 wp_env: "WPEnv",
                 running_timeout_factor=1.1,
                 collision_reward_factor = 1.1,
                 traj_reward_factor = 0.3,
                 traj_len_thresh = 0.6,
                 traj_theta_std_thresh = 0.5,
                 global_goal_reached_reward = 16,
                 step_base_reward: Optional[float] = 1,
                 ):
        super().__init__(
                 observation_collector= observation_collector,
                 wp_env = wp_env,
                 running_timeout_factor=running_timeout_factor,
                 collision_reward_factor = collision_reward_factor,
                 traj_reward_factor = traj_reward_factor,
                 traj_len_thresh = traj_len_thresh,
                 traj_theta_std_thresh = traj_theta_std_thresh,
                 global_goal_reached_reward = global_goal_reached_reward,
                 step_base_reward = step_base_reward) 
                 


@REWARD_REGISTRY.register
class RewardCalculatorWP2_RULE00():
    """ only use sparse reward
    """
    INFO_KEYS = (
        "times_timeout",
        )
    @configurable
    def __init__(self,
                 observation_collector:ObservationCollectorWP2,
                 wp_env: "WPEnv",
                 reward_on_collision:float = -10,
                 reward_on_goal_reached:float = 3,
                 reward_on_time_cost:float = -0.01,
                 reward_on_timeout: float = -0.3,
                 reward_on_progress_base:float = 0.01,
                 no_progress_reward:bool = True,
                 no_timeout_reward:bool = True,
                 ):
        """
        A class for calculating reward based various rules for waypoint generator.


        :param safe_dist (float): The minimum distance to obstacles or wall that robot is in safe status.
                                  if the robot get too close to them it will be punished. Unit[ m ]
        :param goal_radius (float): The minimum distance to goal that goal position is considered to be reached. 
        """
        self.curr_reward = 0
        self.oc = observation_collector
        self.wp_env  = weakref.proxy(wp_env)


        self.reward_on_collision:float = reward_on_collision
        self.reward_on_goal_reached = reward_on_goal_reached
        self.reward_on_time_cost = reward_on_time_cost
        self.reward_on_timeout_cost = reward_on_timeout

        self.reward_on_progress_base = reward_on_progress_base
        self.no_progress_reward = no_progress_reward
        self.no_timeout_reward =no_timeout_reward


    @classmethod
    def from_config(cls,cfg:CfgNode,observation_collector,wp_env):
        return dict(
            observation_collector=observation_collector,
            wp_env = wp_env,
            reward_on_collision = cfg.REWARD.REWARD_ON_COLLISION,
            reward_on_goal_reached = cfg.REWARD.REWARD_ON_GOAL_REACHED,
            reward_on_time_cost = cfg.REWARD.REWARD_ON_TIME_COST,
            reward_on_timeout = cfg.REWARD.REWARD_ON_TIMEOUT,
            reward_on_progress_base =cfg.REWARD.REWARD_ON_PROGRESS_BASE,
            no_progress_reward = cfg.REWARD.NO_PROGRESS_REWARD,
            no_timeout_reward = cfg.REWARD.NO_TIMEOUT_REWARD
        )


    def _reset(self):
        """
        reset variables related to current step
        """
        self.curr_reward = 0

    def reset_on_episode_start(self):
        self.times_timeout = 0
        self.info = {
            'times_timeout': 0
        }
    def get_reward_info(self):
        return self.info

    def save_info_on_episode_end(self):
        if self.times_timeout == 0 and self.no_timeout_reward:
            self.times_timeout = -1 
        self.info['times_timeout']= self.times_timeout

    def get_reward_goal_reached(self):
        self._reset()
        return self.reward_on_goal_reached

    def cal_reward(self):
        """
        Returns reward and info to the gym environment.
        """
        self._reset()
        if self.oc.important_event == ObservationCollectorWP2.Event.COLLISIONDETECTED:
            self.curr_reward += self.reward_on_collision
        else:
            if not self.no_timeout_reward  and self.oc.important_event == ObservationCollectorWP2.Event.TIMEOUT:
                self.times_timeout +=1
                self.curr_reward += self.reward_on_timeout_cost
            elif not self.no_progress_reward and self.oc.last_dist_subgoal_robot is not None:
                self.curr_reward += self.reward_on_progress_base*(self.oc.last_dist_subgoal_robot-self.oc.dist_subgoal_robot)
        return self.curr_reward       
