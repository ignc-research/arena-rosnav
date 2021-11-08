#! /usr/bin/env python
from operator import is_
from random import randint
import gym
from gym import spaces
from gym.spaces import space
from typing import Union
from stable_baselines3.common.env_checker import check_env
import yaml
from rl_agent.utils.observation_collector import ObservationCollector
from rl_agent.utils.reward import RewardCalculator
from rl_agent.utils.debug import timeit
from task_generator.tasks import ABSTask
import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose2D
from flatland_msgs.srv import StepWorld, StepWorldRequest
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as nav_Path
from std_msgs.msg import Bool, Float32, Int8
import time
import math
from rl_agent.utils.debug import timeit
# for transformations
from tf.transformations import *
import subprocess
from task_generator.task_generator.tasks import *
from arena_plan_msgs.msg import RobotState, RobotStateStamped

class wp3Env(gym.Env):
    """Custom Environment that follows gym interface"""

    def __init__(self, 
                 ns: str,  
                 reward_fnc: str, 
                 is_action_space_discrete, 
                 safe_dist: float = None, 
                 goal_radius: float = 0.1, 
                 max_steps_per_episode=100, 
                 train_mode: bool = True, 
                 debug: bool = False,
                 task_mode: str = "staged",
                 PATHS: dict = dict(),
                 extended_eval: bool = False,
                 *args, **kwargs):
        """Default env
        Flatland yaml node check the entries in the yaml file, therefore other robot related parameters cound only be saved in an other file.
        TODO : write an uniform yaml paser node to handel with multiple yaml files.


        Args:
            task (ABSTask): [description]
            reward_fnc (str): [description]
            train_mode (bool): bool to differ between train and eval env during training
            is_action_space_discrete (bool): [description]
            safe_dist (float, optional): [description]. Defaults to None.
            goal_radius (float, optional): [description]. Defaults to 0.1.
            extended_eval (bool): more episode info provided, no reset when crashing
        """
        super(wp3Env, self).__init__()
        # Define action and observation space
        # They must be gym.spaces objects
        self.ns = ns
        try:
            # given every environment enough time to initialize, if we dont put sleep,
            # the training script may crash.
            ns_int = int(ns.split("_")[1])
            time.sleep(ns_int*2)
        except Exception:
            rospy.logwarn(f"Can't not determinate the number of the environment, training script may crash!")
            pass


        # process specific namespace in ros system
        self.ns_prefix = '' if (ns == '' or ns is None) else '/'+ns+'/'
        
        if not debug:
            if train_mode:
                rospy.init_node(f'train_env_{self.ns}', disable_signals=False)
            else:
                rospy.init_node(f'eval_env_{self.ns}', disable_signals=False)

        self._extended_eval = extended_eval
        self._is_train_mode = rospy.get_param("/train_mode")
        self._is_action_space_discrete = is_action_space_discrete
        self.setup_by_configuration(PATHS['robot_setting'], PATHS['robot_as'])
        # set rosparam
        rospy.set_param("/laser_num_beams", self._laser_num_beams)
        # observation collector
        self.observation_collector = ObservationCollector(
             self.ns, self._laser_num_beams, self._laser_max_range)
        self.observation_space = self.observation_collector.get_observation_space()
        self.time_start = 0

        #global variables for subscriber callbacks
        self.range_circle = 1.5
        self._steps_curr_episode = 0
        self._robot_pose = PoseStamped()
        self._globalPlan = nav_Path()
        self._subgoal = Pose2D()
        self._globalGoal = Pose2D()
        self._ref_wp = PoseStamped()
        self._action_msg = PoseStamped()
        self._last_rewarded_waypoint = PoseStamped()
        self._viz_msg = nav_Path()
        self._viz_points = PoseStamped()
        self._robot_twist = [0]*2
        self.firstTime = 0
        self._previous_time = 0
        self._step_counter = 0
        self._amount_rewarded_waypoints = 0
        self._amount_waypoints_left = 0
        self._circle = np.array([])
        self._curr_gp_point = 0
        self._globalPlanArray = np.array([])
        self._curr_goal = Pose2D()
        self._robot_close_to_goal = False
        self._suggested_action = Pose2D()
        #self._goal_action_count = 0
        #self._post_action_count = False

        # reward calculator
        if safe_dist is None:
            safe_dist = 1.6*self._robot_radius

        self.reward_calculator = RewardCalculator(
            robot_radius=self._robot_radius, safe_dist=1.6*self._robot_radius, goal_radius= goal_radius,
            rule=reward_fnc, extended_eval=self._extended_eval)
        #subscriber to infer callback and out of sleep loop
        #sub robot position and sub global goal 
        self._robot_state_sub = rospy.Subscriber(f'{self.ns_prefix}odom', Odometry, self.cbRobotPosition)            # for robot orientation
        self._ref_wp_sub = rospy.Subscriber(f'{self.ns_prefix}subgoal', PoseStamped, self.cbRefWp)                   # for subgoal position (reference point for circle)
        self._globalGoal = rospy.Subscriber(f'{self.ns_prefix}goal', PoseStamped, self.cbGlobalGoal)                 # to set wp on global goal if circle touces it
        self._globalPlan_sub = rospy.Subscriber(f'{self.ns_prefix}globalPlan', nav_Path, self.cbglobalPlan)
        self._twist_sub = rospy.Subscriber(f'{self.ns_prefix}cmd_vel', Twist, self.cbTwist)                          # to calculate distance traveled 
        #self._map_sub = rospy.Subscriber(f'{self.ns_prefix}map', ,self.cbMap)
        self._wp4train_reached =False

        # action agent publisher
        if self._is_train_mode:
            self.agent_action_pub = rospy.Publisher(f'{self.ns_prefix}wp4train', PoseStamped, queue_size=1)
        else:
            self.agent_action_pub = rospy.Publisher(f'{self.ns_prefix}wp4train_pub', PoseStamped, queue_size=1)
  
        # visualization publisher         
        self.circle_pub = rospy.Publisher(f'{self.ns_prefix}zViz', nav_Path, queue_size=1)                               # create the circle on which the wp are placed for visualization
        
        #observation (suggested action) publisher
        self._suggested_action_pub = rospy.Publisher(f'{self.ns_prefix}suggested_action', Pose2D, queue_size=1)
        #observation (left waypoints) publisher
        self._left_waypoints_pub = rospy.Publisher(f'{self.ns_prefix}left_waypoints', Int8, queue_size=1)

        # service clients
        self._is_train_mode = rospy.get_param("/train_mode")
        if self._is_train_mode:
            self._service_name_step = f'{self.ns_prefix}step_world'
            self._sim_step_client = rospy.ServiceProxy(
            self._service_name_step, StepWorld)

        # instantiate task manager
        self.task = get_predefined_task(
            ns, mode=task_mode, start_stage=kwargs['curr_stage'], PATHS=PATHS)

        self._steps_curr_episode = 0
        self._max_steps_per_episode = max_steps_per_episode
        # for reward to calculate how many actions relative to path length
        self.goal_len = 0
        self._action_count = 0
        # # get observation
        # obs=self.observation_collector.get_observations()

    def cbRobotPosition(self,msg):
        self._robot_pose.pose.position.x = msg.pose.pose.position.x
        self._robot_pose.pose.position.y = msg.pose.pose.position.y
        self._robot_pose.pose.orientation = msg.pose.pose.orientation
 
        
    def cbTwist(self,msg):
        self._robot_twist[0]= msg.linear.x
        self._robot_twist[1] = msg.angular.z    

    def cbglobalPlan(self,msg):
        self._globalPlan = msg

    def cbRefWp(self,msg):
        self._ref_wp = msg

    def cbGlobalGoal(self,msg):
        self._globalGoal.x = msg.pose.position.x
        self._globalGoal.y = msg.pose.position.y

        # for extended eval
        self._action_frequency = 1/rospy.get_param("/robot_action_rate")
        self._last_robot_pose = None
        self._distance_travelled = 0
        self._safe_dist_counter = 0
        self._collisions = 0
        self._in_crash = False

    def setup_by_configuration(self, robot_yaml_path: str, settings_yaml_path: str):
        """get the configuration from the yaml file, including robot radius, discrete action space and continuous action space.

        Args:
            robot_yaml_path (str): [description]
        """
        with open(robot_yaml_path, 'r') as fd:
            robot_data = yaml.safe_load(fd)
            # get robot radius
            for body in robot_data['bodies']:
                if body['name'] == "base_footprint":
                    for footprint in body['footprints']:
                        if footprint['type'] == 'circle':
                            self._robot_radius = footprint.setdefault(
                                'radius', 0.3)*1.05
                        if footprint['radius']:
                            self._robot_radius = footprint['radius']*1.05
            # get laser related information
            for plugin in robot_data['plugins']:
                if plugin['type'] == 'Laser':
                    laser_angle_min = plugin['angle']['min']
                    laser_angle_max = plugin['angle']['max']
                    laser_angle_increment = plugin['angle']['increment']
                    self._laser_num_beams = int(
                        round((laser_angle_max-laser_angle_min)/laser_angle_increment)+1)
                    self._laser_max_range = plugin['range']

        with open(settings_yaml_path, 'r') as fd:
            setting_data = yaml.safe_load(fd)
            #print(settings_yaml_path)
            if self._is_action_space_discrete:
                # self._discrete_actions is a list, each element is a dict with the keys ["name", 'linear','angular']
                self._discrete_acitons = setting_data['robot']['discrete_actions']
                self.action_space = spaces.Discrete(
                    len(self._discrete_acitons))
            else:
                linear_range = setting_data['robot']['continuous_actions']['linear_range']
                angular_range = setting_data['robot']['continuous_actions']['angular_range']
                self.action_space = spaces.Box(low=np.array([linear_range[0], angular_range[0]]),
                                               high=np.array(
                                                   [linear_range[1], angular_range[1]]),
                                               dtype=np.float)

    def clear_costmaps(self):
        bashCommand = "rosservice call /move_base/clear_costmaps"
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        return output, error

    def _calc_distance(self, goal_pos:Pose2D,robot_pos:Pose2D):
         y_relative = goal_pos.y - robot_pos.y
         x_relative = goal_pos.x - robot_pos.x
         rho =  (x_relative**2+y_relative**2)**0.5
         theta = (np.arctan2(y_relative,x_relative)-robot_pos.theta+4*np.pi)%(2*np.pi)-np.pi
         return rho,theta

    def _pub_action(self, action):
        _, obs_dict = self.observation_collector.get_observations()
        dist_robot_goal = obs_dict['goal_in_robot_frame']
        dist_global_sub = obs_dict['global_in_subgoal_frame']
        #transform action which is a waypoint to 2d to calculate distance robot-wp
        wp2d = Pose2D()
        wp2d.x = self._action_msg.pose.position.x
        wp2d.y = self._action_msg.pose.position.y

        robot_pos =  Pose2D()
        robot_pos.x = self._robot_pose.pose.position.x 
        robot_pos.y = self._robot_pose.pose.position.y
    
        #calculate distance between robot and waypoint
        dist_robot_wp = self._calc_distance(wp2d, robot_pos)
        self._action_msg.pose.orientation.z =  0 
        self._action_msg.pose.orientation.w = 1
        self._action_msg.header.frame_id ="map"

        #create circle on which waypoints can be spawned 
        circle = nav_Path()
        circle.header.frame_id ="map"
        circle.header.stamp = rospy.Time.now()
        
        ## Draw circle, bound by action space [-2,2] radians, [-3.1,3.1] --> [-pi,pi]
        i = -2
        j = 0
        while (i < 2):
            point = PoseStamped()
            q = self._robot_pose.pose.orientation
            robot_angle = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            angle_grad = i + robot_angle 
            point.pose.position.x = self._ref_wp.pose.position.x + (self.range_circle*math.cos(angle_grad))         
            point.pose.position.y = self._ref_wp.pose.position.y + (self.range_circle*math.sin(angle_grad))
            point.pose.orientation.w=1
            point.header.frame_id="map"
            circle.poses.append(point)
            # if circle touches the global goal, immediately place subgoal on global goal
            #if ((math.isclose(circle.poses[j].pose.position.x, self._globalGoal.x, rel_tol=0.2)) and (math.isclose(circle.poses[j].pose.position.y, self._globalGoal.y, rel_tol=0.2)) and not (self._action_msg.pose.position.x == self._globalGoal.x and self._action_msg.pose.position.y == self._globalGoal.y)):    
                #self._action_msg.pose.position.x = self._globalGoal.x 
                #self._action_msg.pose.position.y = self._globalGoal.y 
                #self._action_msg.pose.orientation.w = 1
                #self.agent_action_pub.publish(self._action_msg)
                #print("subgoal published #1 at",self._action_msg.pose.position.x,self._action_msg.pose.position.y)
                #self._action_count += 1
            i += 0.2
            j += 1
        
        #visualize circle by publishing all poses to /zViz topic
        self.circle_pub.publish(circle)
        #create array from circle
        self._circle = ObservationCollector.process_global_plan_msg(circle)
       
       # First spawn 
        if self.firstTime < 1:
            q = self._robot_pose.pose.orientation
            robot_angle = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            angle_grad = action[0] + robot_angle
            self.firstTime +=1
            if not (action[0] == -0.5):
                self._action_msg.pose.position.x = self._ref_wp.pose.position.x + (self.range_circle*math.cos(angle_grad))         
                self._action_msg.pose.position.y = self._ref_wp.pose.position.y + (self.range_circle*math.sin(angle_grad))
                self.agent_action_pub.publish(self._action_msg)
                #print("subgoal published #2 at",self._action_msg.pose.position.x,self._action_msg.pose.position.y)
                self._action_count += 1
            #else:
                #print("Info: Waypoint was not moved")
            #print("distance robot to wp: {}".format(dist_robot_wp[0]))
            
        #wait for robot to reach the waypoint first
        #if self._step_counter - self._previous_time > 30:
        if True: #dist_robot_wp[0] < 0.6: #From now on it should be possible to always set a new waypoint.
            self._previous_time = self._step_counter
            _, obs_dict = self.observation_collector.get_observations()
            dist_robot_goal = obs_dict['goal_in_robot_frame']
            
            #dist_robot_goal = np.array([self._robot_pose.x - self._subgoal.x, self._robot_pose.y - self._subgoal.y])
            dist_rg = np.linalg.norm(dist_robot_goal)            
            #todo consider the distance to global path when choosing next optimal waypoint
            #send a goal message (posestamped) as action, remeber to normalize the quaternions (put orientationw as 1) and set the frame id of the goal to "map"! 
            if dist_robot_goal[0] < 2:
                if not (self._action_msg.pose.position.x == self._globalGoal.x and self._action_msg.pose.position.y == self._globalGoal.y):
                    self._action_msg.pose.position.x = self._globalGoal.x 
                    self._action_msg.pose.position.y = self._globalGoal.y 
                    self._action_msg.pose.orientation.w = 1
                    self.agent_action_pub.publish(self._action_msg)
                    #print("subgoal published #3 at",self._action_msg.pose.position.x,self._action_msg.pose.position.y)
                    self._action_count += 1
            else:
                q = self._robot_pose.pose.orientation
                robot_angle = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                angle_grad = action[0] + robot_angle 
                if not (action[0] == -0.5):
                    if action[0] == 0.5:
                        self._action_msg.pose.position.x = self._globalGoal.x         
                        self._action_msg.pose.position.y = self._globalGoal.y   
                        self._action_msg.pose.orientation.w = 1
                    else:
                        self._action_msg.pose.position.x = self._ref_wp.pose.position.x + (self.range_circle*math.cos(angle_grad))         
                        self._action_msg.pose.position.y = self._ref_wp.pose.position.y + (self.range_circle*math.sin(angle_grad))   
                        self._action_msg.pose.orientation.w = 1
                    #print("action message looks like {}".format(self._action_msg))
                    self.agent_action_pub.publish(self._action_msg)
                    #print("subgoal published #4 at",self._action_msg.pose.position.x,self._action_msg.pose.position.y)
                    self._action_count += 1
                #else:
                    #print("Info: Waypoint was not moved")
    def _translate_disc_action(self, action):
        new_action = np.array([])
        #new_action = np.append(new_action, self._discrete_acitons[action]['linear']) # not needed anymore
        new_action = np.append(new_action, self._discrete_acitons[action]['angular'])    
            
        return new_action

    def _calculate_optimal_waypoint(self):
        criticalSection = False
        #inital step size is 5% of glob path length
        stepSize = math.floor(len(self._globalPlanArray)/20)
        #if robot is close to goal (6.5 meters) recommend to move the subgoal onto the goal position
        if (((self._globalGoal.x - self._robot_pose.pose.position.x)**2 + (self._globalGoal.y - self._robot_pose.pose.position.y)**2)**0.5) < 6.5:
            self._robot_close_to_goal = True
            # if waypoint on goal, recomment don't move, otherwise recommend set waypoint onto goal
            if (self._action_msg.pose.position.x == self._globalGoal.x and self._action_msg.pose.position.y == self._globalGoal.y):
                return -0.5
            else:
                return 0.5
        # start at the beginning of the global plan, if new episode started, else start from last known point, also lower stepSize in this case 
        if ((self._curr_goal.x == 0 and self._curr_goal.y == 0 and self._curr_goal.theta == 0) or self._curr_goal.x != self._globalGoal.x or self._curr_goal.y != self._globalGoal.y):
            curr_gp_point = 0
            #save current goal for reference
            self._curr_goal.x, self._curr_goal.y = self._globalGoal.x, self._globalGoal.y
        else:
            curr_gp_point = self._curr_gp_point
            stepSize = math.floor(stepSize/4)
            #print("curr gp coord:", self._globalPlanArray[curr_gp_point])
            #print("curr circle coord:", self._circle[10])
            #time.sleep(10)
        # start roughly in the middle of the circle
        curr_circle_point = self._circle[10] 
        #calculate where the middle of the circle and global plan are clostest together or intersect
        min_dist = np.inf
        #print(self._circle)
        while(True):
            #in case the global plan is recalculated and therefor the point is not on the path anymore, start from the beginning
            if curr_gp_point < 0 or curr_gp_point >= len(self._globalPlanArray):
                curr_gp_point = 0
            #print("GP:", self._globalPlanArray[curr_gp_point], "\ncircle: ",curr_circle_point)
            dist = ((self._globalPlanArray[curr_gp_point][0] - curr_circle_point[0])**2 + (self._globalPlanArray[curr_gp_point][1] - curr_circle_point[1])**2)**0.5
            if dist < min_dist:
                min_dist = dist
                #criticalSection == False means you overshot only once, but this time you're closer again
                if criticalSection:
                    optimal_angle = wp3Env._calculate_closest_angle(self, curr_gp_point - math.floor(stepSize/2), curr_gp_point)
                    break
                #only step forward, if there is global path to step on, otherwise step onto last point of global plan
                if curr_gp_point + stepSize < len(self._globalPlanArray):
                    curr_gp_point += stepSize
                else:
                    curr_gp_point = len(self._globalPlanArray) -1
            else:
                #critialSection == True means you overshot twice at this point
                if criticalSection:
                    optimal_angle = wp3Env._calculate_closest_angle(self, curr_gp_point - stepSize, curr_gp_point)
                    break
                criticalSection = True
                curr_gp_point -= math.floor(stepSize/2)

                #go half way back. If you're still further away, you overshot the intersection. 
                # so you need to search between now (which is half way back) and at most 1 way back (worst case) from here => between curr_gp_point and curr_gp_point -= stepSize)
                
                #if you go back half way and you're closer, then it is between current position and another half way back => curr_gp_point and curr_gp_point -= floor(stepSize/2)
        #save last used gp point minus one step to variable to start from there later
        #print("current point:", curr_gp_point)
        self._curr_gp_point = curr_gp_point - stepSize
        return optimal_angle

    def _calculate_closest_angle(self, gp_point_1, gp_point_2):
        #print(gp_point_2 - gp_point_1)
        min_dist = np.inf
        min_dist_circle_index = 0
        for i in range(len(self._circle)):
            for j in range(gp_point_2 - gp_point_1):
                circle_gp_dist = ((self._globalPlanArray[gp_point_1+j][0] - self._circle[i][0])**2 + (self._globalPlanArray[gp_point_1+j][1] - self._circle[i][1])**2)**0.5
                #print(i, min_dist, circle_gp_dist)
                if circle_gp_dist < min_dist:
                    min_dist = circle_gp_dist
                    min_dist_circle_index = i
        #print(min_dist)            
        return round(-2 + 0.2 * min_dist_circle_index,2)

    def step(self, action):
        """
        done_reasons:   0   -   exceeded max steps
                        1   -   collision with obstacle
                        2   -   goal reached
        """
        if self._is_action_space_discrete:
            action = self._translate_disc_action(action)
        self._pub_action(action)
        ###DEBUG###
        #if action == 0.5:
        #    self._goal_action_count += 1
        #    self._post_action_count = True
        #if self._post_action_count:
        #    print(self._goal_action_count)
        #    self._post_action_count = False
        #test what would happen, if robot always uses suggested action
        #self._pub_action(np.array([self._suggested_action]))
        self._globalPlanArray = ObservationCollector.process_global_plan_msg(self._globalPlan)
        if len(self._globalPlanArray) > 0 and len(self._circle) > 0:
            self._suggested_action.x = wp3Env._calculate_optimal_waypoint(self)
            #print(self._suggested_action.x)
            self._suggested_action_pub.publish(self._suggested_action)
            #print("suggested action:", self._suggested_action)
        if self._steps_curr_episode == 0:
            self.time_start = time.time() #reward will be calculated based on the elapsed time. Time starts right after publishing the action and stops when first reward gets calculated
        self._steps_curr_episode += 1

        ##todo: wait for robot_pos = goal pos

        #in each step, get the robots cmd velocity to get action for reward distance traveled
        new_action = [0]*2
        new_action[0] = self._robot_twist[0]
        new_action[1] = self._robot_twist[1]
        # wait for new observations
        merged_obs, obs_dict = self.observation_collector.get_observations()
        # print("get observation: {}".format(time.time()-s))

        # get global path once new episode started and round to integer for reward waypoints_set relative to goal distance
        if self.firstTime < 2:
            self.goal_len = int(round(obs_dict['goal_in_robot_frame'][0]))
        # provide reward calculator with the data to calculate reward
        reward, reward_info = self.reward_calculator.get_reward(
            obs_dict['laser_scan'], obs_dict['goal_in_robot_frame'],
            robot_pose=obs_dict['robot_pose'], 
            global_plan=self._globalPlanArray, 
            action=new_action, 
            time_elapsed = time.time() - self.time_start, #time which elapsed since the 1st action has been published
            goal=self._globalGoal,
            subgoal=self._action_msg,
            last_subgoal = self._last_rewarded_waypoint,
            amount_rewarded_subgoals = self._amount_rewarded_waypoints,
            robot_close_to_goal = self._robot_close_to_goal
            )
        
        if reward_info['subgoal_was_rewarded']:
            self._last_rewarded_waypoint.pose.position.x = self._action_msg.pose.position.x
            self._last_rewarded_waypoint.pose.position.y = self._action_msg.pose.position.y
            self._amount_rewarded_waypoints += 1

        self._amount_waypoints_left = 1 if reward_info['estimated_subgoals'] == 100000 else max(0,reward_info['estimated_subgoals'] - self._amount_rewarded_waypoints)
        self._left_waypoints_pub.publish(self._amount_waypoints_left)
        done = reward_info['is_done']
        #print("reward:  {}".format(reward))
                # extended eval info
        if self._extended_eval:
            self._update_eval_statistics(obs_dict, reward_info)
        # info
        info = {}
        if done:
            info['done_reason'] = reward_info['done_reason']
            info['is_success'] = reward_info['is_success']
            self.reward_calculator.kdtree = None
            info['gp_len'] = len(self._globalPlanArray)

        if self._steps_curr_episode > self._max_steps_per_episode:
            done = True
            info['done_reason'] = 0
            info['is_success'] = 0
            self.reward_calculator.kdtree = None
            info['gp_len'] = len(self._globalPlanArray)

                # for logging
        if self._extended_eval:
            if done:
                info['collisions'] = self._collisions
                info['distance_travelled'] = round(self._distance_travelled, 2)
                info['time_safe_dist'] = self._safe_dist_counter * self._action_frequency
                info['time'] = self._steps_curr_episode * self._action_frequency       
        return merged_obs, reward, done, info

    def reset(self):
        #self.clear_costmaps()

        # set task
        # regenerate start position end goal position of the robot and change the obstacles accordingly
        #self.agent_action_pub.publish(Twist())
        if self._is_train_mode:
            self._sim_step_client()
        self.task.reset()
        self.reward_calculator.reset()
        self._steps_curr_episode = 0
        self.firstTime = 0
        self._action_count = 0
        self._last_rewarded_waypoint = PoseStamped()
        self._amount_rewarded_waypoints = 0
        self._amount_waypoints_left = 0
        self._robot_close_to_goal = False
        self._curr_gp_point = 0
        self._suggested_action = Pose2D()
        # extended eval info
        if self._extended_eval:
            self._last_robot_pose = None
            self._distance_travelled = 0
            self._safe_dist_counter = 0
            self._collisions = 0
        obs, _ = self.observation_collector.get_observations()
        return obs  # reward, done, info can't be included

    def close(self):
        pass

def _update_eval_statistics(self, obs_dict: dict, reward_info: dict):
        """
        Updates the metrics for extended eval mode

        param obs_dict (dict): observation dictionary from ObservationCollector.get_observations(),
            necessary entries: 'robot_pose'
        param reward_info (dict): dictionary containing information returned from RewardCalculator.get_reward(),
            necessary entries: 'crash', 'safe_dist'
        """
        # distance travelled
        if self._last_robot_pose is not None:
            self._distance_travelled += wp3Env.get_distance(
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

@staticmethod
def get_distance(pose_1: Pose2D, pose_2: Pose2D):
    return math.hypot(pose_2.x - pose_1.x, pose_2.y - pose_1.y)

if __name__ == '__main__':

    rospy.init_node('wp3_gym_env', anonymous=True, disable_signals=False)
    print("start")

    wp3_env = wp3Env()
    check_env(wp3_env, warn=True)

    # init env
    obs = wp3_env.reset()

    # run model
    n_steps = 200
    for step in range(n_steps):
        # action, _states = model.predict(obs)
        action = wp3_env.action_space.sample()

        obs, rewards, done, info = wp3_env.step(action)

        time.sleep(0.1)
