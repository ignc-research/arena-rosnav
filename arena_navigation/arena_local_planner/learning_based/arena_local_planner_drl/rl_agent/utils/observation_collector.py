#! /usr/bin/env python
import threading
from typing import Tuple
from numpy.core.numeric import normalize_axis_tuple
import rospy
import rospkg
import random
import numpy as np
from collections import deque
import yaml
import os
import time  # for debuging
import threading
import copy
# observation msgs
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, PoseStamped, PoseWithCovarianceStamped 
from geometry_msgs.msg import Twist 
from pedsim_msgs.msg import AgentState
from arena_plan_msgs.msg import RobotState, RobotStateStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock
import math

# services
from flatland_msgs.srv import StepWorld, StepWorldRequest

# message filter
import message_filters

# for transformations
from tf.transformations import *

from gym import spaces
import numpy as np

from std_msgs.msg import Bool

from rl_agent.utils.debug import timeit 

class ObservationCollector():
    def __init__(self, ns: str, start_stage:int):
        """ a class to collect and merge observations

        Args:
            num_lidar_beams (int): [description]
            lidar_range (float): [description]
        """
        self.ns = ns
        if ns is None or ns == "":
            self.ns_prefix = "/"
        else:
            self.ns_prefix = "/"+ns+"/"
        
        self.safe_dists_human_type = self.read_saftey_distance_parameter_from_yaml()['human obstacle safety distance radius']
        self.safe_dists_robot_type = self.read_saftey_distance_parameter_from_yaml()['robot obstacle safety distance radius']
        self.safe_dists_factor = self.read_saftey_distance_parameter_from_yaml()['safety distance factor']
        self.obstacle_radius = self.read_saftey_distance_parameter_from_yaml()['obstacle radius']
        self.human_behavior_tokens= copy.deepcopy(self.safe_dists_factor)
        for i,item in enumerate(list(self.human_behavior_tokens.items())):
            self.human_behavior_tokens[ item[0]] = i
        self.human_type_ids= copy.deepcopy(self.safe_dists_human_type)
        for i,item in enumerate(list(self.human_type_ids.items())):
            self.human_type_ids[ item[0]] = i
        self.robo_type_ids= copy.deepcopy(self.safe_dists_robot_type)
        for i,item in enumerate(list(self.robo_type_ids.items())):
            self.robo_type_ids[ item[0]] = i




        self._radius_robot= 0.3

        #get parameters
        self._laser_num_beams = rospy.get_param("/laser_num_beams")  # for frequency controlling
        self._action_frequency = 1/rospy.get_param("/robot_action_rate")
        self._is_train_mode = rospy.get_param("/train_mode")   # train mode?
        # service clients
        if self._is_train_mode:
            self._service_name_step = f'{self.ns_prefix}step_world'
            self._sim_step_client = rospy.ServiceProxy(
                self._service_name_step, StepWorld)

        #define functions as variables 
        self._clock = Clock()
        self._scan = LaserScan()
        self._robot_pose = Pose2D()
        self._robot_vel = Twist()
        self._subgoal = Pose2D()
        self._globalplan = np.array([])
        
        self.first_obs = True
        self.last = 0
        self.last_r = 0
        #subscribtions
        self._scan_sub = message_filters.Subscriber( f'{self.ns_prefix}scan', LaserScan)  #subscribe to robot scan 
        self._robot_state_sub = message_filters.Subscriber(f'{self.ns_prefix}robot_state', RobotStateStamped) #subscribe to robot state 
        self._subgoal_sub = rospy.Subscriber(f'{self.ns_prefix}subgoal', PoseStamped, self.callback_subgoal)#subscribe to subgoal
        self._globalplan_sub = rospy.Subscriber(f'{self.ns_prefix}globalPlan', Path, self.callback_global_plan) #subscribe to gloabalPlan
        self._sub_next = rospy.Subscriber(f"{self.ns_prefix}next_stage", Bool, self.subscribe_obstacles_topics)
        self._sub_prev = rospy.Subscriber(f"{self.ns_prefix}previous_stage", Bool, self.subscribe_obstacles_topics)
        self.subscribe_obstacles_topics(True)
 

    def subscribe_obstacles_topics(self,msg:Bool):
        

        while  rospy.get_param("/_reseting_obstacles") == True : 
            print('*******************waiting for _reseting_obstacles **********************')
        self.curr_stage = rospy.get_param("/curr_stage", -1)
        self.agent_state=[]
        self.robo_obstacle_state=[]
        #human state subscriper
        self.num_humans= self.read_stages_from_yaml()[self.curr_stage]['dynamic_human']
        for i in range(self.num_humans):
            self.agent_state.append(f'{self.ns_prefix}pedsim_agent_{i+1}/agent_state') #making a a list of the topics names 
        print('topics are ',self.agent_state)
        self._sub_agent_state=[None]*self.num_humans
        for i, topic in enumerate(self.agent_state):
            self._sub_agent_state[i]=message_filters.Subscriber(topic, AgentState) #subscribing to the topics of every Agent
        #robots state subscriper
        self.num_robo_obstacles=self.read_stages_from_yaml()[self.curr_stage]['dynamic_robot']
        for i in range( self.num_robo_obstacles):
            if i <10 :
             self.robo_obstacle_state.append(f'{self.ns_prefix}robo_obstacle_0{i+1}') #making a a list of the topics names 
            else:
                self.robo_obstacle_state.append(f'{self.ns_prefix}/robo_obstacle_{i+1}') #making a a list of the topics names 
        self._sub_robo_obstacles_state=[None]* self.num_robo_obstacles
        for i, topic in enumerate(self.robo_obstacle_state):
            self._sub_robo_obstacles_state[i]=message_filters.Subscriber(topic, Marker) #subscribing to the topics of every robo osbstacle
        
        #intilasing arrays for human states
        self._human_type=np.array( [None]*self.num_humans)
        self._human_position=np.array( [None]*self.num_humans)
        self._human_vel=np.array( [None]*self.num_humans)
        self._human_behavior=np.array( [None]*self.num_humans)

        #intilasing arrays for robo states
        self._robo_obstacle_type=np.array( [None]* self.num_robo_obstacles)
        self._robo_obstacle_position=np.array( [None]* self.num_robo_obstacles)
        self._robo_obstacle_vel=np.array( [None]* self.num_robo_obstacles)
 
        #synchronization parameters
        self._first_sync_obs = True     # whether to return first sync'd obs or most recent
        self.max_deque_size = 10
        self._sync_slop = 0.05
        self._laser_deque = deque()
        self._rs_deque = deque()

        # message_filters.TimeSynchronizer: call callback only when all sensor info are ready
        self.sychronized_list=[self._scan_sub, self._robot_state_sub]+self._sub_agent_state  + self._sub_robo_obstacles_state #[self._scan_sub, self._robot_state_sub]+self._adult+self._child+self._elder
        self.ts = message_filters.ApproximateTimeSynchronizer(self.sychronized_list, 10, slop=0.1) 
        self.ts.registerCallback(self.callback_observation_received)
        rospy.set_param("/_initiating_stage", False)  

    def get_observation_space(self):
        return self.observation_space

    def get_observations(self):
        # apply action time horizon

        
        self._flag_all_received=False
        if self._is_train_mode: 
        # sim a step forward until all sensor msg uptodate
           # i=0
            while(self._flag_all_received==False):
                # self._action_frequency
                # print('entered locking area')
                self.call_service_takeSimStep(0.1)
            #    i+=1
                time.sleep(0.01)
            # print(f"Current observation takes {i} steps for Synchronization")
        else:
            try:
                rospy.wait_for_message(
                    f"{self.ns_prefix}next_cycle", Bool)
            except Exception:
                pass

        if len(self._scan.ranges) > 0:
            scan = self._scan.ranges.astype(np.float32)
        else:
            scan = np.ones(self._laser_num_beams, dtype=float)*100
            
        #claculating diffrent robot infos 
        rho, theta = ObservationCollector._get_pose_in_robot_frame(self._subgoal, self._robot_pose)
        self.rot=np.arctan2(self._subgoal.y - self._robot_pose.y, self._subgoal.x - self._robot_pose.x)
        self.robot_vx = self._robot_vel.linear.x * np.cos(self.rot) + self._robot_vel.linear.y * np.sin(self.rot)
        self.robot_vy=self._robot_vel.linear.y* np.cos(self.rot) - self._robot_vel.linear.x * np.sin(self.rot)
        self.robot_self_state=[self._robot_pose.x, self._robot_pose.y, self.robot_vx, self.robot_vy,
                                                     self._robot_pose.theta, self._robot_vel.angular.z, self._radius_robot, rho, theta]
        merged_obs = np.hstack([np.array([self.time_step]), scan])
        obs_dict = {}
        obs_dict["robot_velocity"]= math.sqrt(self._robot_vel.linear.x*self._robot_vel.linear.x+self._robot_vel.linear.y*self._robot_vel.linear.y)
        obs_dict["laser_scan"] = scan
        obs_dict['goal_in_robot_frame'] = [rho,theta]
        # initlaising array with dimensions an filling them up with coordinate of agents and rho(density)and theta (angle)
        count_observable_humans=0  
        obs_dict['human_coordinates_in_robot_frame']= []
        obs_dict['human_obstacles_in_robot_frame'] = np.array([],dtype=object).reshape(0, 4)

        agent_massage_is_none = False
        for pos in self._human_position: 
            if pos  is None  : 
                print(self.ns_prefix,'´´´´´´´´´´ERORR got Agent Massage with None´´´´´´´´´')
                agent_massage_is_none = True
            elif math.isnan(pos.x )  == True  : 
                print(self.ns_prefix,'´´´´´´´´´´ERORR got Agent Massage with Nan´´´´´´´´´')
                agent_massage_is_none = True


        if len(self._human_position) == self.num_humans and self.num_humans > 0  and agent_massage_is_none == False :
            rho_humans, theta_humans=np.empty([self.num_humans,]), np.empty([self.num_humans,])
            coordinate_humans= np.empty([2,self.num_humans])

            for  i, position in enumerate(self._human_position):
                #TODO temporarily use the same fnc of _get_pose_in_robot_frame (finished)
            
                coordinate_humans[0][i]=position.x
                coordinate_humans[1][i]=position.y
                rho_humans[i], theta_humans[i] = ObservationCollector._get_pose_in_robot_frame(position, self._robot_pose)

            #sort the humans according to the relative position to robot
            human_pos_index=np.argsort(rho_humans)
            rho_humans, theta_humans=rho_humans[human_pos_index], theta_humans[human_pos_index]
            self._human_type=self._human_type[human_pos_index]
            self._human_vel=self._human_vel[human_pos_index]
            self._human_position=self._human_position[human_pos_index]
            self._human_behavior=self._human_behavior[human_pos_index]
            # add them to obs_dict
            obs_dict['human_coordinates_in_robot_frame']=coordinate_humans
            obs_dict['human_type']=self._human_type
            obs_dict['human_behavior']=self._human_behavior


            
            
            
            for i, ty in enumerate(self._human_type):
                # filter the obstacles which are not in the visible range of the robot
                if not self.IsInViewRange(15, [-np.pi, np.pi], rho_humans[i], theta_humans[i]):
                    continue
                else:
                    count_observable_humans=count_observable_humans+1
                    
                    rho_behavior=np.array([rho_humans[i],self._human_behavior[i],self._human_type[i],self.human_type_ids[self._human_type[i]]],dtype=object)
                    
                    obs_dict['human_obstacles_in_robot_frame'] = np.vstack([obs_dict['human_obstacles_in_robot_frame'], rho_behavior])
                    #determine the safe_dist for every human
                    safe_dist_=self.safe_dists_human_type[ty] * self.safe_dists_factor[self._human_behavior[i]]
                    _radius =self.obstacle_radius[ty]
                    _human_behavior_token=self.human_behavior_tokens[self._human_behavior[i]]
                    #robot centric 4 elements in state
                    state=ObservationCollector.rotate(self.robot_self_state[:2]+[self._human_position[i].x, self._human_position[i].y, self._human_vel[i].linear.x,self._human_vel[i].linear.y], self.rot)
                    obs=np.array(self.robot_self_state+[rho_humans[i], theta_humans[i]]+state+[safe_dist_ ,_radius,
                                                    _radius+safe_dist_+self._radius_robot,_human_behavior_token])
                    
                    merged_obs = np.hstack([merged_obs,obs])
        
     
 
        if count_observable_humans==0:
            obs_empty=np.array(self.robot_self_state+[0]*10)
            merged_obs = np.hstack([merged_obs,obs_empty])
            count_observable_humans=count_observable_humans+1
        while count_observable_humans < 5 and count_observable_humans >0:
            obs_copy=np.copy(merged_obs[-count_observable_humans*self.human_state_size:])
            merged_obs = np.hstack([merged_obs, obs_copy])
            count_observable_humans=count_observable_humans*2
        
        #align the observation size
        observation_blank=len(merged_obs) - self.observation_space.shape[0] +self.num_robo_obstacles_observation_max*self.robo_obstacle_state_size
        if observation_blank<0:
            #add invalid value 1000 if the merged_obs are not full
            merged_obs=np.hstack([merged_obs,np.ones([-observation_blank,])*1000])
        elif observation_blank>0:
            merged_obs=merged_obs[:-observation_blank]

      # initlaising array with dimensions an filling them up with coordinate of agents and rho(density)and theta (angle)
        rho_robo_obstacles, theta_robo_obstacles=np.empty([self.num_robo_obstacles,]), np.empty([self.num_robo_obstacles,])
        coordinate_robo_obstacles= np.empty([2,self.num_robo_obstacles])
        # print("robo",self._robo_obstacle_position)
        count_observable_robo_obstacles=0
        obs_dict['robo_obstacle_coordinates_in_robot_frame']=[]
        obs_dict['robot_obstacles_in_robot_frame'] = np.array([],dtype=object).reshape(0,3)
        for pos in self._robo_obstacle_position: 
            if pos  is None  : 
                print(self.ns_prefix,'´´´´´´´´´´ERORR got robo Agent Massage with None´´´´´´´´´')
                agent_massage_is_none = True
            elif math.isnan(pos.x )  == True  : 
                print(self.ns_prefix,'´´´´´´´´´´ERORR got  Agent Massage with Nan´´´´´´´´´')
                time.sleep(1)
                agent_massage_is_none = True

        if len(self._robo_obstacle_position) == self.num_robo_obstacles  and self.num_robo_obstacles > 0 and agent_massage_is_none == False  :
            for  i, position in enumerate(self._robo_obstacle_position):
                #TODO temporarily use the same fnc of _get_pose_in_robot_frame (finished)

                coordinate_robo_obstacles[0][i]=position.x
                coordinate_robo_obstacles[1][i]=position.y
                rho_robo_obstacles[i], theta_robo_obstacles[i] = ObservationCollector._get_pose_in_robot_frame(position, self._robot_pose)

            #sort the humans according to the relative position to robot
            robo_obstacles_pos_index=np.argsort(rho_robo_obstacles)
            rho_robo_obstacles, theta_robo_obstacles=rho_robo_obstacles[robo_obstacles_pos_index], theta_robo_obstacles[robo_obstacles_pos_index]
            self._robo_obstacle_type =self._robo_obstacle_type[robo_obstacles_pos_index]
            self._robo_obstacle_vel=self._robo_obstacle_vel[robo_obstacles_pos_index]
            self._robo_obstacle_position=self._robo_obstacle_position[robo_obstacles_pos_index]
        
            # add them to obs_dict
            obs_dict['robo_obstacle_coordinates_in_robot_frame']=coordinate_robo_obstacles
            obs_dict['robo_obstacle_type']=self._robo_obstacle_type
           
            

            rho_behavior_randomwandrer = np.array([],dtype=object).reshape(0, 1) 

            count_observable_robo_obstacles= 0
            for i, ty in enumerate(self._robo_obstacle_type):
                # filter the obstacles which are not in the visible range of the robot
                if not self.IsInViewRange(20, [-2.618,2.618], rho_robo_obstacles[i], theta_robo_obstacles[i]):
                    continue
                else:
                    count_observable_robo_obstacles = count_observable_robo_obstacles +1
                    
                    rho_behavior=np.array([rho_robo_obstacles[i],self._robo_obstacle_type[i],self.robo_type_ids[ self._robo_obstacle_type[i]]],dtype=object)
                    obs_dict['robot_obstacles_in_robot_frame'] = np.vstack([obs_dict['robot_obstacles_in_robot_frame'], rho_behavior])
                    #determine the safe_dist for every robot
                    safe_dist_=self.safe_dists_robot_type[ty] 
                    _radius =self.obstacle_radius[ty]
                    state=ObservationCollector.rotate(self.robot_self_state[:2]+[self._robo_obstacle_position[i].x, self._robo_obstacle_position[i].y, self._robo_obstacle_vel[i].x,self._robo_obstacle_vel[i].y], self.rot)
                    obs=np.array(self.robot_self_state+[rho_robo_obstacles[i], theta_robo_obstacles[i]]+state+[safe_dist_ ,_radius, _radius+safe_dist_+self._radius_robot,-1])
                    merged_obs = np.hstack([merged_obs,obs])

        #TODO more proper method is needed to supplement info blanks (finished)
        if count_observable_robo_obstacles==0:
            obs_empty=np.array(self.robot_self_state+[0]*10)
            merged_obs = np.hstack([merged_obs,obs_empty])
            count_observable_robo_obstacles=count_observable_robo_obstacles+1
        while count_observable_robo_obstacles < 2 and count_observable_robo_obstacles >0:
            obs_copy=np.copy(merged_obs[-count_observable_robo_obstacles*self.robo_obstacle_state_size:])
            merged_obs = np.hstack([merged_obs, obs_copy])
            count_observable_robo_obstacles=count_observable_robo_obstacles*2
        
        #align the observation size
        observation_blank=len(merged_obs) - self.observation_space.shape[0]
        if observation_blank<0:
            #add invalid value 1000 if the merged_obs are not full
            merged_obs=np.hstack([merged_obs,np.ones([-observation_blank,])*1000])
        elif observation_blank>0:
            merged_obs=merged_obs[:-observation_blank]
        

        return merged_obs, obs_dict
   
    @staticmethod
    def _get_pose_in_robot_frame(agent_pos: Pose2D, robot_pos: Pose2D):
        y_relative = agent_pos.y - robot_pos.y
        x_relative = agent_pos.x - robot_pos.x
        rho =  np.linalg.norm([y_relative, x_relative])
        theta = 0
     
        theta = (np.arctan2(y_relative, x_relative) -
                robot_pos.theta+5*np.pi) % (2*np.pi)-np.pi
    
        return rho, theta



    @staticmethod
    def is_synchronized(self, msg_Laser: LaserScan, msg_Robotpose: RobotStateStamped):
        laser_stamp = round(msg_Laser.header.stamp.to_sec(), 4)
        robot_stamp = round(msg_Robotpose.header.stamp.to_sec(), 4)

        return bool(laser_stamp == robot_stamp)

    def get_sync_obs(self):
        laser_scan = None
        robot_pose = None
        #print(f"laser deque: {len(self._laser_deque)}, robot state deque: {len(self._rs_deque)}")
        while len(self._rs_deque) > 0 and len(self._laser_deque) > 0:
            laser_scan_msg = self._laser_deque.popleft()
            robot_pose_msg = self._rs_deque.popleft()
            
            laser_stamp = laser_scan_msg.header.stamp.to_sec()
            robot_stamp = robot_pose_msg.header.stamp.to_sec()

            while not abs(laser_stamp - robot_stamp) <= self._sync_slop:
                if laser_stamp > robot_stamp:
                    if len(self._rs_deque) == 0:
                        return laser_scan, robot_pose
                    robot_pose_msg = self._rs_deque.popleft()
                    robot_stamp = robot_pose_msg.header.stamp.to_sec()
                else:
                    if len(self._laser_deque) == 0:
                        return laser_scan, robot_pose
                    laser_scan_msg = self._laser_deque.popleft()
                    laser_stamp = laser_scan_msg.header.stamp.to_sec()

            laser_scan = self.process_scan_msg(laser_scan_msg)
            robot_pose, _ = self.process_robot_state_msg(robot_pose_msg)

            if self._first_sync_obs:
                break        
        #print(f"Laser_stamp: {laser_stamp}, Robot_stamp: {robot_stamp}")
        return laser_scan, robot_pose

    def call_service_takeSimStep(self, t=None):
        if t is None:
            request = StepWorldRequest()
        else:
            request = StepWorldRequest(t)
        try:
            response = self._sim_step_client(request)
            rospy.logdebug("step service=", response)
        except rospy.ServiceException as e:
            rospy.logdebug("step Service call failed: %s" % e)


    def callback_clock(self, msg_Clock):
        self._clock = msg_Clock.clock.to_sec()
        return

    def callback_subgoal(self, msg_Subgoal):
        self._subgoal = self.process_subgoal_msg(msg_Subgoal)
        return

    def callback_global_plan(self, msg_global_plan):
        self._globalplan = ObservationCollector.process_global_plan_msg(msg_global_plan)
        return

    def callback_scan(self, msg_laserscan):
        if len(self._laser_deque) == self.max_deque_size:
            self._laser_deque.popleft()
        self._laser_deque.append(msg_laserscan)

    def callback_robot_state(self, msg_robotstate):
        if len(self._rs_deque) == self.max_deque_size:
            self._rs_deque.popleft()
        self._rs_deque.append(msg_robotstate)

    def callback_observation_received(self, *msg):
        self._scan=self.process_scan_msg(msg[0])
        self._robot_pose,self._robot_vel=self.process_robot_state_msg(msg[1])
        self.callback_agent_state(msg[2:self.num_humans+2])
        self.callback_robo_obstacle_state(msg[self.num_humans+2:])
        # print("calling back")

        self._flag_all_received=True

    def callback_agent_state(self, msg):
        for i, m in enumerate(msg):
            self._human_type[i],self._human_position[i],self._human_vel[i], self._human_behavior[i]=self.process_agent_state(m)

    def callback_robo_obstacle_state(self, msg):
        # print("calling back massages ",msg)
        for i, m in enumerate(msg):
            self._robo_obstacle_type[i],self._robo_obstacle_position[i],self._robo_obstacle_vel[i]=self.process_robo_obstacle_state(m)
        # print("recieved",self._robo_obstacle_type)

    def process_agent_state(self,msg):
        human_type=msg.type
        human_pose=self.pose3D_to_pose2D(msg.pose)   
        human_twist=msg.twist
        human_behavior=msg.social_state.strip("\"")
 
        return human_type,human_pose, human_twist, human_behavior 
    
    def process_robo_obstacle_state(self,msg):
        robo_obstacle_type=msg.ns
        robo_obstacle_pose=self.pose3D_to_pose2D(msg.pose)
        robo_twist=msg.scale
        return robo_obstacle_type,robo_obstacle_pose,robo_twist

    def process_scan_msg(self, msg_LaserScan: LaserScan):
        # remove_nans_from_scan
        self._scan_stamp = msg_LaserScan.header.stamp.to_sec()
        scan = np.array(msg_LaserScan.ranges)
        scan[np.isnan(scan)] = msg_LaserScan.range_max
        msg_LaserScan.ranges = scan
        return msg_LaserScan

    def process_robot_state_msg(self, msg_RobotStateStamped: RobotStateStamped):
        self._rs_stamp = msg_RobotStateStamped.header.stamp.to_sec()
        state = msg_RobotStateStamped.state
        pose3d = state.pose
        twist = state.twist
        return self.pose3D_to_pose2D(pose3d), twist

    def process_human_state_msg(self,msg_humanodom):
        pose=self.process_pose_msg(msg_humanodom)
        # pose3d=state.pose
        twist=msg_humanodom.twist.twist
        return pose, twist

    def process_pose_msg(self, msg_PoseWithCovarianceStamped):
        # remove Covariance
        pose_with_cov = msg_PoseWithCovarianceStamped.pose
        pose = pose_with_cov.pose
        return self.pose3D_to_pose2D(pose)

    def process_subgoal_msg(self, msg_Subgoal):
        pose2d = self.pose3D_to_pose2D(msg_Subgoal.pose)
        return pose2d

    def set_timestep(self, time_step):
        self.time_step=time_step

    def setRobotSettings(self, num_lidar_beams, lidar_range, laser_angle_min, laser_angle_max, laser_angle_increment):
        self.num_lidar_beams = num_lidar_beams
        self.lidar_range = lidar_range
        self.laser_angle_min = laser_angle_min
        self.laser_angle_max = laser_angle_max
        self.laser_angle_increment = laser_angle_increment
        self.laser_beam_angles = np.arange(self.laser_angle_min, self.laser_angle_max, self.laser_angle_increment)

    def setObservationSpace(self):
        # define observation_space
        self.num_humans_observation_max=21
        self.human_state_size=19
        self.num_robo_obstacles_observation_max=10
        self.robo_obstacle_state_size=19 
        self.observation_space = ObservationCollector._stack_spaces((
            spaces.Box(low=-np.PINF, high=np.PINF, shape=(1,),dtype=np.float64), #time
            spaces.Box(low=0.0, high=self.lidar_range, shape=(self.num_lidar_beams,),dtype=np.float64), #lidar
            spaces.Box(low=-np.PINF, high=np.PINF, shape=(self.num_humans_observation_max*self.human_state_size,),dtype=np.float64), # human states
            spaces.Box(low=-np.PINF, high=np.PINF, shape=(self.num_robo_obstacles_observation_max*self.robo_obstacle_state_size,),dtype=np.float64) # human states

        ))

    def IsInViewRange(self, distance, angleRange, rho_human, theta_human):
        if rho_human<=distance and theta_human<=angleRange[1] and theta_human>=angleRange[0]:
            return True
        else:
            return False

    def calculateDangerZone(self, vx, vy):
        a = 0.55
        r_static = 0.8
        v = np.linalg.norm([vx, vy])
        radius = a*v+ r_static
        theta = 11*np.pi/6* np.exp(-1.4*v)+ np.pi/6
        return radius, theta

    def read_saftey_distance_parameter_from_yaml(self):
        
        file_location = os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'saftey_distance_parameter.yaml')
        
        
        if os.path.isfile(file_location):
            with open(file_location, "r") as file:
                saftey_distance_parameter = yaml.load(file, Loader=yaml.FullLoader)       
        assert isinstance(
             saftey_distance_parameter, dict), "'saftey_distance_parameter.yaml' has wrong fromat! Has to encode dictionary!"
                
        return saftey_distance_parameter

    def read_stages_from_yaml(self):
        dir = rospkg.RosPack().get_path('arena_local_planner_drl')
        
        
        file_location = os.path.join( dir, 'configs', 'training_curriculum.yaml')
        
        if os.path.isfile(file_location):
            with open(file_location, "r") as file:
                _stages = yaml.load(file, Loader=yaml.FullLoader)
            assert isinstance(
                _stages, dict), "'training_curriculum.yaml' has wrong fromat! Has to encode dictionary!"
            
        else:
            raise FileNotFoundError(
                "Couldn't find 'training_curriculum.yaml' in %s " % file_location)
        return _stages
        
    @staticmethod
    def process_global_plan_msg(globalplan):
        global_plan_2d = list(map(
            lambda p: ObservationCollector.pose3D_to_pose2D(p.pose), globalplan.poses))
        global_plan_np = np.array(list(map(lambda p2d: [p2d.x,p2d.y], global_plan_2d)))
        return global_plan_np

    @staticmethod
    def pose3D_to_pose2D(pose3d):
        pose2d = Pose2D()
        pose2d.x = pose3d.position.x
        pose2d.y = pose3d.position.y
        quaternion = (pose3d.orientation.x, pose3d.orientation.y,
                      pose3d.orientation.z, pose3d.orientation.w)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        pose2d.theta = yaw
        return pose2d

    @staticmethod
    def _stack_spaces(ss: Tuple[spaces.Box]):
        low = []
        high = []
        for space in ss:
            low.extend(space.low.tolist())
            high.extend(space.high.tolist())
        return spaces.Box(np.array(low).flatten(), np.array(high).flatten(),dtype=np.float64)

    @staticmethod
    def rotate(state, rot):
        """
        Transform the coordinate to agent-centric.
        """
        # 'px', 'py','px1', 'py1', 'vx1', 'vy1'
        #   0       1       2        3         4        5 
        vx1 = state[4] * np.cos(rot) + state[5] * np.sin(rot)
        vy1 = state[5] * np.cos(rot) - state[4] * np.sin(rot)
        px1 = (state[2] - state[0]) * np.cos(rot) + (state[3] - state[1]) * np.sin(rot)

        py1 = (state[3] - state[1]) * np.cos(rot) - (state[2] - state[0]) * np.sin(rot)

        new_state = [px1,py1,vx1,vy1]
        return new_state


if __name__ == '__main__':

    rospy.init_node('states', anonymous=True)
    print("start")

    state_collector = ObservationCollector("sim1/", 360, 10)
    i = 0
    r = rospy.Rate(100)
    while(i <= 1000):
        i = i+1
        obs = state_collector.get_observations()

        time.sleep(0.001)
