#! /usr/bin/env python
import threading
from typing import Tuple

from numpy.core.numeric import normalize_axis_tuple
import rospy
import random
import numpy as np
from collections import deque

import time  # for debuging
import threading
# observation msgs
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from pedsim_msgs.msg import AgentState
from flatland_msgs.msg import DangerZone
from arena_plan_msgs.msg import RobotState, RobotStateStamped
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock

# services
from flatland_msgs.srv import StepWorld, StepWorldRequest

# message filter
import message_filters

# for transformations
from tf.transformations import *

from gym import spaces
import numpy as np

from std_msgs.msg import Bool

# from rl_agent.utils.debug import timeit 

class ObservationCollector():
    def __init__(self, ns: str, num_humans:int):
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

        #safety settings for different humans TODO: should be global setting
        self.safe_dist_adult = 1.0 #in meter 1.0 1.5 2
        self.safe_dist_child = 1.2
        self.safe_dist_elder = 1.5
        self.safe_dist_talking = 0.8
        #settings for agents TODO: should be transfered from yaml files
        self._radius_adult= 0.32
        self._radius_child= 0.25
        self._radius_elder= 0.3
        self._radius_robot= 0.3
        self._radius_human_average=0.3


        self._laser_num_beams = rospy.get_param("/laser_num_beams")
        # for frequency controlling
        # self._action_frequency = 1/rospy.get_param("/robot_action_rate")
        self.useDangerZone = rospy.get_param("/useDangerZone")


        self._clock = Clock()
        self._scan = LaserScan()
        self._robot_pose = Pose2D()
        self._robot_vel = Twist()
        self._subgoal = Pose2D()
        self._globalplan = np.array([])

        # train mode?
        self._is_train_mode = rospy.get_param("/train_mode")

        # synchronization parameters
        self._first_sync_obs = True     # whether to return first sync'd obs or most recent
        self.max_deque_size = 10
        self._sync_slop = 0.05

        self._laser_deque = deque()
        self._rs_deque = deque()

        # subscriptions
        # self._scan_sub = rospy.Subscriber(
        #     f'{self.ns_prefix}scan', LaserScan, self.callback_scan, tcp_nodelay=True)

        # self._robot_state_sub = rospy.Subscriber(
        #     f'{self.ns_prefix}robot_state', RobotStateStamped, self.callback_robot_state, tcp_nodelay=True)
        self._scan_sub = message_filters.Subscriber( f'{self.ns_prefix}scan', LaserScan)
        self._robot_state_sub = message_filters.Subscriber(f'{self.ns_prefix}robot_state', RobotStateStamped)
        
        # self._clock_sub = rospy.Subscriber(
        #     f'{self.ns_prefix}clock', Clock, self.callback_clock, tcp_nodelay=True)
       
        self._subgoal_sub = rospy.Subscriber(
            f'{self.ns_prefix}subgoal', PoseStamped, self.callback_subgoal)

        self._globalplan_sub = rospy.Subscriber(
            f'{self.ns_prefix}globalPlan', Path, self.callback_global_plan)

        # service clients
        if self._is_train_mode:
            self._service_name_step = f'{self.ns_prefix}step_world'
            self._sim_step_client = rospy.ServiceProxy(
                self._service_name_step, StepWorld)

        self.first_obs = True
        self.last = 0
        self.last_r = 0
        self.agent_state=[]
        self.dangerZone=[]
        #human state subscriber
        self.num_humans=num_humans
        for i in range(num_humans):
            if self.useDangerZone:
                self.dangerZone.append(f'{self.ns_prefix}pedsim_agent_{i+1}/danger_zone')
            self.agent_state.append(f'{self.ns_prefix}pedsim_agent_{i+1}/agent_state')
        self._sub_danger_zone=[None]*self.num_humans
        self._sub_agent_state=[None]*num_humans
        self._human_type, self._human_position, self._human_vel= [None]*num_humans,[None]*num_humans,[None]*num_humans
        self._human_behavior, self._safe_dist, self._dangerAngle=[None]*self.num_humans,[None]*self.num_humans,[None]*self.num_humans
        self._dangerCenter=[None]*self.num_humans
        self._human_type =np.array(self._human_type)
        self._human_position=np.array(self._human_position)
        self._human_vel=np.array(self._human_vel)
        self._human_behavior=np.array(self._human_behavior)
        #declaration of danger zone infos
        self._safe_dist=np.array(self._safe_dist)
        self._dangerAngle=np.array(self._dangerAngle)
        self._dangerCenter=np.array(self._dangerCenter)
        for i, topic in enumerate(self.agent_state):
            if self.useDangerZone:
                self._sub_danger_zone[i]=message_filters.Subscriber(self.dangerZone[i], DangerZone)
            self._sub_agent_state[i]=message_filters.Subscriber(topic, AgentState)

        # message_filters.TimeSynchronizer: call callback only when all sensor info are ready
        if self.useDangerZone:
            self.sychronized_list=[self._scan_sub, self._robot_state_sub]+self._sub_agent_state + self._sub_danger_zone #[self._scan_sub, self._robot_state_sub]+self._adult+self._child+self._elder
        else:
            self.sychronized_list=[self._scan_sub, self._robot_state_sub]+self._sub_agent_state #[self._scan_sub, self._robot_state_sub]+self._adult+self._child+self._elder
        self.ts = message_filters.ApproximateTimeSynchronizer(self.sychronized_list, 10, slop=0.01) #,allow_headerless=True)
        self.ts.registerCallback(self.callback_observation_received)

    def get_observation_space(self):
        return self.observation_space

    def get_observations(self):
        # apply action time horizon
        self._flag_all_received=False
        i_train = 0
        i_time = 0.0
        if self._is_train_mode:
            # sim a step forward until all sensor msg uptodate            
            while(self._flag_all_received==False):
                # self._action_frequency
                # print('entered locking area')
                self.call_service_takeSimStep(0.1)
                i_train +=  1
                time.sleep(0.01)
                i_time += 0.11
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
            
        rho, theta = ObservationCollector._get_pose_in_robot_frame(
            self._subgoal, self._robot_pose)
        # print( self.ns_prefix+'_goal_robot_theta', theta)
        self.rot=np.arctan2(self._subgoal.y - self._robot_pose.y, self._subgoal.x - self._robot_pose.x)
        self.robot_vx = self._robot_vel.linear.x * np.cos(self.rot) + self._robot_vel.linear.y * np.sin(self.rot)
        self.robot_vy=self._robot_vel.linear.y* np.cos(self.rot) - self._robot_vel.linear.x * np.sin(self.rot)
        self.robot_self_state=[self._robot_pose.x, self._robot_pose.y, self.robot_vx, self.robot_vy,
                                                     self._robot_pose.theta, self._robot_vel.angular.z, self._radius_robot, rho, theta]
        merged_obs = np.hstack([np.array([self.time_step]), scan])
        obs_dict = {}
        obs_dict["laser_scan"] = scan
        obs_dict['goal_in_robot_frame'] = [rho,theta]
        obs_dict['robot_pose'] = self._robot_pose

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

        rho_adult = np.array([])
        rho_child = np.array([])
        rho_elder = np.array([])

        for i, ty in enumerate(self._human_type):
            if ty==0: # adult
                rho_adult=np.hstack([rho_adult, rho_humans[i]])
            elif ty==1: # child
                rho_child=np.hstack([rho_child, rho_humans[i]])
            elif ty==3: # elder
                rho_elder=np.hstack([rho_elder, rho_humans[i]])

        obs_dict['adult_distances'] = rho_adult
        obs_dict['child_distances'] = rho_child
        obs_dict['elder_distances'] = rho_elder

        obs_dict['human_coordinates_in_robot_frame']=coordinate_humans
        obs_dict['human_type']=self._human_type
        obs_dict['human_behavior']=self._human_behavior

        # semantic tokens TODO: different tokens for other behaviors
        self._human_behavior_token=(self._human_behavior=='talking').astype(np.int)

        rho_behavior_adult = np.array([],dtype=object).reshape(0, 2)
        rho_behavior_child = np.array([],dtype=object).reshape(0, 2)
        rho_behavior_elder = np.array([],dtype=object).reshape(0, 2)

        isInDangerZone_adult = np.array([],dtype=bool).reshape(0,1)
        isInDangerZone_child = np.array([],dtype=bool).reshape(0,1)
        isInDangerZone_elder = np.array([],dtype=bool).reshape(0,1)
        RF_And_Dc_adult = np.array([],dtype=float).reshape(0,2)
        RF_And_Dc_child = np.array([],dtype=float).reshape(0,2)
        RF_And_Dc_elder = np.array([],dtype=float).reshape(0,2)

        count_observable_humans=0
        for i, ty in enumerate(self._human_type):
            # filter the obstacles which are not in the visible range of the robot
            if not self.isInViewRange(15, [-np.pi, np.pi], rho_humans[i], theta_humans[i]):
                continue
            else:
                count_observable_humans=count_observable_humans+1

            rho_behavior=np.array([rho_humans[i],self._human_behavior[i]],dtype=object)
            state_human_in_robot_frame=ObservationCollector.rotate(self.robot_self_state[:2]+[self._human_position[i].x, self._human_position[i].y, self._human_vel[i].linear.x,self._human_vel[i].linear.y], self.rot)

            if ty==0: # adult
                rho_behavior_adult=np.vstack([rho_behavior_adult, rho_behavior])
                #determine the safe_dist for every human
                if self._human_behavior[i] =='talking':
                    safe_dist_=self.safe_dist_talking
                else:
                    safe_dist_=self.safe_dist_adult

                #robot centric 
                if not self.useDangerZone:
                    obs=np.array(self.robot_self_state+[rho_humans[i], theta_humans[i]]+state_human_in_robot_frame+[safe_dist_ ,self._radius_adult,
                                                    self._radius_adult+safe_dist_+self._radius_robot,self._human_behavior_token[i]])
                    merged_obs = np.hstack([merged_obs,obs])
                else:
                    angle=self._get_robot_pose_in_human_frame(self._dangerCenter[i], self._human_position[i])
                    # state_human_in_robot_frame=ObservationCollector.rotate(self.robot_self_state[:2]+[self._human_position[i].x, self._human_position[i].y, self._human_vel[i].linear.x,self._human_vel[i].linear.y], self.rot)
                    rot_human=np.arctan2(self._subgoal.y - self._dangerCenter[i][1], self._subgoal.x - self._dangerCenter[i][0])
                    state_robot_in_human_frame=ObservationCollector.rotate([self._human_position[i].x, self._human_position[i].y,self._robot_pose.x, self._robot_pose.y, self._robot_vel.linear.x, self._robot_vel.linear.y], self.rot) #rot_human
                    obs=np.array(self.robot_self_state+[rho_humans[i], theta_humans[i]]+state_human_in_robot_frame+state_robot_in_human_frame+[self._safe_dist[i],self._radius_human_average,
                                                    self._radius_human_average+self._safe_dist[i]+self._radius_robot, angle, -self._dangerAngle[i]/2, self._dangerAngle[i]/2])
                    # print('single size', obs.shape[0])
                    merged_obs = np.hstack([merged_obs,obs])
                    isIn=self.isInDangerZone(rho_humans[i],self._safe_dist[i], angle, self._dangerAngle[i])
                    # print(i, 'isin danger', isIn)
                    isInDangerZone_adult=np.vstack([isInDangerZone_adult,isIn])
                    RF_And_Dc_Row=np.array([self._safe_dist[i]-self._radius_human_average,rho_humans[i]-self._radius_human_average-self._radius_robot]).reshape(1,2)
                    RF_And_Dc_adult=np.vstack([RF_And_Dc_adult,RF_And_Dc_Row])
            elif ty==1: # child
                # rho_behavior=np.array([rho_humans[i],self._human_behavior[i]],dtype=object)
                rho_behavior_child=np.vstack([rho_behavior_child, rho_behavior])
                #determine the safe_dist for every human
                if self._human_behavior[i] =='talking':
                    safe_dist_=self.safe_dist_talking
                else:
                    safe_dist_=self.safe_dist_child
                #robot centric 
                if not self.useDangerZone:
                # state=ObservationCollector.rotate(self.robot_self_state[:2]+[self._human_position[i].x, self._human_position[i].y, self._human_vel[i].linear.x,self._human_vel[i].linear.y],self.rot)
                    obs=np.array(self.robot_self_state+[rho_humans[i], theta_humans[i]]+state_human_in_robot_frame+[safe_dist_,self._radius_child,
                                                    self._radius_child+safe_dist_+self._radius_robot,self._human_behavior_token[i]])
                    merged_obs = np.hstack([merged_obs,obs])
                else:
                    angle=self._get_robot_pose_in_human_frame(self._dangerCenter[i], self._human_position[i])
                    rot_human=np.arctan2(self._subgoal.y - self._dangerCenter[i][1], self._subgoal.x - self._dangerCenter[i][0])
                    state_robot_in_human_frame=ObservationCollector.rotate([self._human_position[i].x, self._human_position[i].y,self._robot_pose.x, self._robot_pose.y, self._robot_vel.linear.x, self._robot_vel.linear.y], self.rot) #rot_human
                    obs=np.array(self.robot_self_state+[rho_humans[i], theta_humans[i]]+state_human_in_robot_frame+state_robot_in_human_frame+[self._safe_dist[i],self._radius_human_average,
                                                    self._radius_human_average+self._safe_dist[i]+self._radius_robot, angle, -self._dangerAngle[i]/2, self._dangerAngle[i]/2])
                    merged_obs = np.hstack([merged_obs,obs])
                    isIn=self.isInDangerZone(rho_humans[i],self._safe_dist[i], angle, self._dangerAngle[i])
                    isInDangerZone_child=np.vstack([isInDangerZone_child,isIn])
                    RF_And_Dc_Row=np.array([self._safe_dist[i]-self._radius_human_average,rho_humans[i]-self._radius_human_average-self._radius_robot]).reshape(1,2)
                    RF_And_Dc_child=np.vstack([RF_And_Dc_child,RF_And_Dc_Row])

            elif ty==3: # elder
                # rho_behavior=np.array([rho_humans[i],self._human_behavior[i]],dtype=object)
                rho_behavior_elder=np.vstack([rho_behavior_elder, rho_behavior])
                #determine the safe_dist for every human
                if self._human_behavior[i] =='talking':
                    safe_dist_=self.safe_dist_talking
                else:
                    safe_dist_=self.safe_dist_elder
                #robot centric 
                if not self.useDangerZone:
                # state=ObservationCollector.rotate(self.robot_self_state[:2]+[self._human_position[i].x, self._human_position[i].y, self._human_vel[i].linear.x,self._human_vel[i].linear.y], self.rot)
                    obs=np.array(self.robot_self_state+[rho_humans[i], theta_humans[i]]+state_human_in_robot_frame+[safe_dist_,self._radius_elder,
                                                    self._radius_elder+safe_dist_+self._radius_robot, self._human_behavior_token[i]])
                    merged_obs = np.hstack([merged_obs,obs])
                else:
                    angle=self._get_robot_pose_in_human_frame(self._dangerCenter[i], self._human_position[i])
                    rot_human=np.arctan2(self._subgoal.y - self._dangerCenter[i][1], self._subgoal.x - self._dangerCenter[i][0])
                    state_robot_in_human_frame=ObservationCollector.rotate([self._human_position[i].x, self._human_position[i].y,self._robot_pose.x, self._robot_pose.y, self._robot_vel.linear.x, self._robot_vel.linear.y], self.rot) #rot_human
                    obs=np.array(self.robot_self_state+[rho_humans[i], theta_humans[i]]+state_human_in_robot_frame+state_robot_in_human_frame+[self._safe_dist[i],self._radius_human_average,
                                                    self._radius_human_average+self._safe_dist[i]+self._radius_robot, angle, -self._dangerAngle[i]/2, self._dangerAngle[i]/2])
                    merged_obs = np.hstack([merged_obs,obs])
                    isIn=self.isInDangerZone(rho_humans[i],self._safe_dist[i], angle, self._dangerAngle[i])
                    isInDangerZone_elder=np.vstack([isInDangerZone_elder,isIn])
                    RF_And_Dc_Row=np.array([self._safe_dist[i]-self._radius_human_average,rho_humans[i]-self._radius_human_average-self._radius_robot]).reshape(1,2)
                    RF_And_Dc_elder=np.vstack([RF_And_Dc_elder,RF_And_Dc_Row])
                    
        obs_dict['adult_in_robot_frame'] = rho_behavior_adult
        obs_dict['child_in_robot_frame'] = rho_behavior_child
        obs_dict['elder_in_robot_frame'] = rho_behavior_elder
        if self.useDangerZone:
            obs_dict['danger_zone_adult'] = isInDangerZone_adult
            obs_dict['danger_zone_child'] = isInDangerZone_child
            obs_dict['danger_zone_elder'] = isInDangerZone_elder
            obs_dict['RF_and_Dc_adult'] = RF_And_Dc_adult
            obs_dict['RF_and_Dc_child'] = RF_And_Dc_child
            obs_dict['RF_and_Dc_elder'] = RF_And_Dc_elder
        #TODO more proper method is needed to supplement info blanks (finished)
        if count_observable_humans==0:
            obs_empty=np.array(self.robot_self_state+[0]*10)
            merged_obs = np.hstack([merged_obs,obs_empty])
            count_observable_humans=count_observable_humans+1
        while count_observable_humans < 6 and count_observable_humans >0:
            obs_copy=np.copy(merged_obs[-count_observable_humans*self.human_state_size:])
            merged_obs = np.hstack([merged_obs, obs_copy])
            count_observable_humans=count_observable_humans*2
        #align the observation size
        observation_blank=len(merged_obs) - self.observation_space.shape[0]
        if observation_blank<0:
            #add invalid value 1000 if the merged_obs are not full
            merged_obs=np.hstack([merged_obs,np.ones([-observation_blank,])*1000])
        elif observation_blank>0:
            merged_obs=merged_obs[:-observation_blank]        
        return merged_obs, obs_dict, (i_train, i_time)

    @staticmethod
    def _get_pose_in_robot_frame(agent_pos: Pose2D, robot_pos: Pose2D):
        y_relative = agent_pos.y - robot_pos.y
        x_relative = agent_pos.x - robot_pos.x
        rho =  np.linalg.norm([y_relative, x_relative])
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
        if self.useDangerZone:
            self.callback_danger_zone(msg[self.num_humans+2:])
        self._flag_all_received=True

    def callback_danger_zone(self, msg):
            for i, m in enumerate(msg):
                self._safe_dist[i] = m.dangerZoneRadius+self._radius_human_average #radius of human
                self._dangerAngle[i]=m.dangerZoneAngle
                self._dangerCenter[i]=m.dangerZoneCenter

    def callback_agent_state(self, msg):
            for i, m in enumerate(msg):
                self._human_type[i],self._human_position[i],self._human_vel[i], self._human_behavior[i]=self.process_agent_state(m)

    def process_agent_state(self,msg):
        human_type=msg.type
        human_pose=self.pose3D_to_pose2D(msg.pose)
        human_twist=msg.twist
        human_behavior=msg.social_state.strip("\"")
        return human_type,human_pose, human_twist, human_behavior

    def process_scan_msg(self, msg_LaserScan: LaserScan):
        # remove_nans_from_scan
        self._scan_stamp = msg_LaserScan.header.stamp.to_sec()
        scan = np.array(msg_LaserScan.ranges)
        # print(self.ns_prefix+'scan', scan[89], scan[179], scan[269])
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
        if self.useDangerZone:
            self.human_state_size=25
        else:
            self.human_state_size=19
        self.observation_space = ObservationCollector._stack_spaces((
            spaces.Box(low=-np.PINF, high=np.PINF, shape=(1,),dtype=np.float64), #time
            spaces.Box(low=0.0, high=self.lidar_range, shape=(self.num_lidar_beams,),dtype=np.float64), #lidar
            spaces.Box(low=-np.PINF, high=np.PINF, shape=(self.num_humans_observation_max*self.human_state_size,),dtype=np.float64) # human states
        ))

    def isInViewRange(self, distance, angleRange, rho_human, theta_human):
        if rho_human<=distance and theta_human<=angleRange[1] and theta_human>=angleRange[0]:
            return True
        else:
            return False

    def isInDangerZone(self, distance, safeDistance, angle, angleRange):
        # angle=self._get_robot_pose_in_human_frame(center, human_pos)
        if distance < safeDistance and (angle >= -angleRange/2 or angle <= angleRange/2):
            return True
        else:
            return False

    def _get_robot_pose_in_human_frame(self, center, human_pos: Pose2D):
        y_relative = self._robot_pose.y - center[1]
        x_relative = self._robot_pose.x - center[0]
        theta = (np.arctan2(y_relative, x_relative) -
                 human_pos.theta+5*np.pi) % (2*np.pi)-np.pi
        return theta

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
