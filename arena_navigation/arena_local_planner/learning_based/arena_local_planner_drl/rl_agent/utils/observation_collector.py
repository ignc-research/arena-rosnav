#! /usr/bin/env python
import threading
from typing import Tuple

from numpy.core.numeric import normalize_axis_tuple
import rospy
import random
import numpy as np

import time  # for debuging
import threading
# observation msgs
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from pedsim_msgs.msg import AgentState
from arena_plan_msgs.msg import RobotState,RobotStateStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
# services
from flatland_msgs.srv import StepWorld,StepWorldRequest
from std_srvs.srv import Trigger, TriggerRequest

# message filter
import message_filters

# for transformations
from tf.transformations import *

from gym import spaces
import numpy as np


class ObservationCollector():
    def __init__(self,ns: str, num_lidar_beams:int,lidar_range:float, num_humans:int): #
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
        self.safe_dist_adult=0.8 #in meter 1.0 1.5 2
        self.safe_dist_child=1.2
        self.safe_dist_elder=1.5
        #settings for agents TODO: should be transferred from yaml files
        self._radius_adult= 0.32
        self._radius_child= 0.25
        self._radius_elder= 0.3
        self._radius_robot= 0.3

        # define observation_space
        self.num_humans_observation_max=21
        self.observation_space = ObservationCollector._stack_spaces((
            spaces.Box(low=0.0, high=lidar_range, shape=(num_lidar_beams,),dtype=np.float64),
            spaces.Box(low=0.0, high=10.0, shape=(1,),dtype=np.float64) ,
            spaces.Box(low=-np.pi, high=np.pi, shape=(1,),dtype=np.float64),
            spaces.Box(low=-np.PINF, high=np.PINF, shape=(7,),dtype=np.float64),
            spaces.Box(low=-np.PINF, high=np.PINF, shape=(self.num_humans_observation_max*10,),dtype=np.float64)
        ))

        self._scan = LaserScan()
        self._robot_pose = Pose2D()
        self._robot_vel = Twist()
        self._subgoal =  Pose2D()
        
        # topic subscriber: subgoal
        #TODO should we synchoronize it with other topics
        self._subgoal_sub = message_filters.Subscriber( f'{self.ns_prefix}subgoal', PoseStamped)#self._subgoal_sub = rospy.Subscriber("subgoal", PoseStamped, self.callback_subgoal)
        self._subgoal_sub.registerCallback(self.callback_subgoal)

        # service clients
        get_train_mode_try=0
        max_try=10
        while(get_train_mode_try<max_try):            
            try:
                self._is_train_mode = rospy.get_param("/train_mode")
                break
            except KeyError:
                get_train_mode_try+=1
                print(f'value not set retry {get_train_mode_try} times')

        if self._is_train_mode:
            self._service_name_step=f'{self.ns_prefix}step_world' 
            self._sim_step_client = rospy.ServiceProxy(self._service_name_step, StepWorld)
        
        # message_filter subscriber: laserscan, robot_pose
        self._scan_sub = message_filters.Subscriber( f'{self.ns_prefix}scan', LaserScan)
        self._robot_state_sub = message_filters.Subscriber(f'{self.ns_prefix}robot_state', RobotStateStamped)
        get_agent_topic_try=0
        max_try=10
        while(get_agent_topic_try<max_try):            
            try:
                self.human_name_str=rospy.get_param(f'{self.ns_prefix}agent_topic_string')
                break
            except KeyError:
                get_agent_topic_try+=1
                print(f'value not set retry {get_agent_topic_try} times')

        self.human_name_list=self.human_name_str.split(',')[1:]
        
        self.agent_state=[]
        self.num_humans=num_humans
        for i in range(num_humans):
            self.agent_state.append(f'{self.ns_prefix}pedsim_agent_{i+1}/agent_state')
        self._sub_agent_state=[None]*num_humans
        self._human_type, self._human_position, self._human_vel= [None]*num_humans,[None]*num_humans,[None]*num_humans
        self._human_type =np.array(self._human_type)
        self._human_position=np.array(self._human_position)
        self._human_vel=np.array(self._human_vel)
        for i, topic in enumerate(self.agent_state):
            self._sub_agent_state[i]=message_filters.Subscriber(topic, AgentState)

        # message_filters.TimeSynchronizer: call callback only when all sensor info are ready
        self.sychronized_list=[self._scan_sub, self._robot_state_sub]+self._sub_agent_state #[self._scan_sub, self._robot_state_sub]+self._adult+self._child+self._elder
        self.ts = message_filters.ApproximateTimeSynchronizer(self.sychronized_list,10,slop=0.1) #,allow_headerless=True)        
        self.ts.registerCallback(self.callback_observation_received)

    
    def get_observation_space(self):
        return self.observation_space

    def get_observations(self):
        def all_sub_received():
            ans = True
            for k, v in self._sub_flags.items():
                if v is not True:
                    ans = False
                    break
            return ans

        def reset_sub():
            self._sub_flags = dict((k, False) for k in self._sub_flags.keys())
        self._flag_all_received=False
        if self._is_train_mode: 
        # sim a step forward until all sensor msg uptodate
            i=0
            while(self._flag_all_received==False):
                self.call_service_takeSimStep()
                i+=1
        # with self._sub_flags_con:
        #     while not all_sub_received():
        #         self._sub_flags_con.wait()  # replace it with wait for later
        #     reset_sub()
        # rospy.logdebug(f"Current observation takes {i} steps for Synchronization")
        # print(f"Current observation takes {i} steps for Synchronization")
        scan = np.array(self._scan.ranges)
        # print("scan",scan)
        rho, theta = ObservationCollector._get_goal_pose_in_robot_frame(
            self._subgoal, self._robot_pose)        
        # self.rot=np.arctan2(self._subgoal.y-self._robot_pose.y, self._subgoal.x-self._robot_pose.x)
        self.robot_self_state=[self._robot_vel.linear.x, self._robot_vel.linear.y, self._robot_pose.x, self._robot_pose.y,
                                                     self._robot_pose.theta, self._robot_vel.angular.z]
        merged_obs = np.hstack([scan, np.array([rho, theta]+self.robot_self_state+[self.time_step])])
        obs_dict = {}
        obs_dict["laser_scan"] = scan
        obs_dict['goal_in_robot_frame'] = [rho,theta]

        rho_humans,theta_humans=np.empty([self.num_humans,]),np.empty([self.num_humans,])
        coordinate_humans= np.empty([2,self.num_humans])
        for  i, position in enumerate(self._human_position):
            #TODO temporarily use the same fnc of _get_goal_pose_in_robot_frame
            coordinate_humans[0][i]=position.x
            coordinate_humans[1][i]=position.y
            rho_humans[i], theta_humans[i] = ObservationCollector._get_goal_pose_in_robot_frame(position,self._robot_pose)
        #sort the humans according to the relative position to robot
        human_pos_index=np.argsort(rho_humans)        
        rho_humans, theta_humans=rho_humans[human_pos_index], theta_humans[human_pos_index]
        self._human_type=self._human_type[human_pos_index]
        self._human_vel=self._human_vel[human_pos_index]
        self._human_position=self._human_position[human_pos_index]
        obs_dict['human_coordinates_in_robot_frame']=coordinate_humans
        obs_dict['human_type']=self._human_type

        rho_adult=[]
        rho_child=[]
        rho_elder=[]

        for i, ty in enumerate(self._human_type):            
            if ty==0: # adult
                rho_adult.append(rho_humans[i])
                #robot centric 
                # state=rotate(self.robot_self_state[:4]+[self._human_position[i].x, self._human_position[i].y, self._human_vel[i].linear.x,self._human_vel[i].linear.y])
                obs=np.array([rho_humans[i], theta_humans[i], self._human_vel[i].linear.x,self._human_vel[i].linear.y, self._human_position[i].x, self._human_position[i].y,
                                                self._radius_adult+self.safe_dist_adult+self._radius_robot, ty])
                merged_obs = np.hstack([merged_obs,obs])
            elif ty==1: # child
                rho_child.append(rho_humans[i])
                #10 states of other agents
                obs=np.array([rho_humans[i], theta_humans[i], self._human_vel[i].linear.x,self._human_vel[i].linear.y, self._human_position[i].x, self._human_position[i].y,
                                              self.safe_dist_child, self._radius_child,self._radius_child+self._radius_robot, ty])
                merged_obs = np.hstack([merged_obs,obs])
            elif ty==3: # elder
                rho_elder.append(rho_humans[i])
                # print('human elder',rho_humans[i])
                #10 states of other agents
                obs=np.array([rho_humans[i], theta_humans[i], self._human_vel[i].linear.x,self._human_vel[i].linear.y, self._human_position[i].x, self._human_position[i].y,
                                              self.safe_dist_elder, self._radius_elder,self._radius_elder+self._radius_robot, ty])
                merged_obs = np.hstack([merged_obs,obs])
        # print(len(rho_adult))
        obs_dict['adult_in_robot_frame'] = np.array(rho_adult)
        obs_dict['child_in_robot_frame'] = np.array(rho_child)
        obs_dict['elder_in_robot_frame'] = np.array(rho_elder)
        #align the observation size
        # print(self.observation_space.shape)
        observation_blank=len(merged_obs) - self.observation_space.shape[0]
        if observation_blank<0:
            merged_obs=np.hstack([merged_obs,np.zeros([-observation_blank,])])
        elif observation_blank>0:
            merged_obs=merged_obs[:-observation_blank]

        return merged_obs, obs_dict

    @staticmethod
    def _get_goal_pose_in_robot_frame(goal_pos: Pose2D, robot_pos: Pose2D):
        y_relative = goal_pos.y - robot_pos.y
        x_relative = goal_pos.x - robot_pos.x
        rho = (x_relative**2+y_relative**2)**0.5
        theta = (np.arctan2(y_relative, x_relative) - robot_pos.theta+5*np.pi) % (2*np.pi)-np.pi
        return rho, theta

    def call_service_takeSimStep(self):
        # print("add one step")
        request = StepWorldRequest()
        try:
            response = self._sim_step_client(request)
            rospy.logdebug("step service=", response)
        except rospy.ServiceException as e:
            rospy.logdebug("step Service call failed: %s" % e)

    def callback_subgoal(self,msg_Subgoal):
        self._subgoal=self.process_subgoal_msg(msg_Subgoal)        
        return

    def callback_dynamic_obstacles(self,msg_human):
        # print("reached here callback human")
        num_adult = len(self._adult)
        num_child = len(self._child)
        num_elder = len(self._elder)
        msg_adult=msg_human[:num_adult]
        msg_child=msg_human[num_adult:num_adult+num_child]
        msg_elder=msg_human[num_adult+num_child:]
        for i,msg in enumerate(msg_adult):
            # print("x",msg.pose.pose.position.x)
            self._adult_position[i],self._adult_vel[i]=self.process_human_state_msg(msg_adult[i])
        for i,msg in enumerate(msg_child):
            self._child_position[i],self._child_vel[i]=self.process_human_state_msg(msg_child[i])
        for i,msg in enumerate(msg_elder):
            self._elder_position[i],self._elder_vel[i]=self.process_human_state_msg(msg_elder[i])
        return
        
    def callback_observation_received(self, *msg):
        # process sensor msg
        # print("reached here callback")
        self._scan=self.process_scan_msg(msg[0])
        self._robot_pose,self._robot_vel=self.process_robot_state_msg(msg[1])
        self.callback_agent_state(msg[2:])
        #self.callback_dynamic_obstacles(msg[2:])
        # ask subgoal service
        #self._subgoal=self.call_service_askForSubgoal()
        self._flag_all_received=True
    def callback_agent_state(self, msg):
            for i, m in enumerate(msg):
                self._human_type[i],self._human_position[i],self._human_vel[i]=self.process_agent_state(m)
    
    def process_agent_state(self,msg):
        human_type=msg.type
        human_pose=self.pose3D_to_pose2D(msg.pose)
        human_twist=msg.twist
        return human_type,human_pose, human_twist

    def process_scan_msg(self, msg_LaserScan):
        # remove_nans_from_scan
        scan = np.array(msg_LaserScan.ranges)
        scan[np.isnan(scan)] = msg_LaserScan.range_max
        msg_LaserScan.ranges = scan
        return msg_LaserScan

    def process_robot_state_msg(self, msg_RobotStateStamped):
        state = msg_RobotStateStamped.state
        pose3d = state.pose
        twist = state.twist
        return self.pose3D_to_pose2D(pose3d), twist

    def process_human_state_msg(self,msg_humanodom):
        pose=self.process_pose_msg(msg_humanodom)
        # pose3d=state.pose
        twist=msg_humanodom.twist.twist
        return pose, twist
        
    def process_pose_msg(self,msg_PoseWithCovariance):
        # remove Covariance
        pose_with_cov=msg_PoseWithCovariance.pose
        pose=pose_with_cov.pose
        return self.pose3D_to_pose2D(pose)

    def process_subgoal_msg(self, msg_Subgoal):
        pose2d = self.pose3D_to_pose2D(msg_Subgoal.pose)
        return pose2d

    def set_timestep(self, time_step):
        self.time_step=time_step

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
        return spaces.Box(np.array(low).flatten(), np.array(high).flatten(), dtype=np.float64)



if __name__ == '__main__':

    rospy.init_node('states', anonymous=True)
    print("start")

    state_collector = ObservationCollector("sim_1/", 360, 10)
    i = 0
    r = rospy.Rate(100)
    while(i <= 1000):
        i = i+1
        obs = state_collector.get_observations()

        time.sleep(0.001)
