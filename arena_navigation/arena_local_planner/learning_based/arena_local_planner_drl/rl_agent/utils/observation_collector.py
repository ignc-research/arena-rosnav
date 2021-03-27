#! /usr/bin/env python
from typing import Tuple

from numpy.core.numeric import normalize_axis_tuple
import rospy
import random
import numpy as np
from collections import deque
import time # for debuging

# observation msgs
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D,PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from arena_plan_msgs.msg import RobotState, RobotStateStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

# services
from flatland_msgs.srv import StepWorld,StepWorldRequest


# message filter
import message_filters

# for transformations
from tf.transformations import *

from gym import spaces
import numpy as np




class ObservationCollector():
    def __init__(self,num_lidar_beams:int,lidar_range:float):
        """ a class to collect and merge observations

        Args:
            num_lidar_beams (int): [description]
            lidar_range (float): [description]
        """
        # define observation_space
        self.observation_space = ObservationCollector._stack_spaces((
            spaces.Box(low=0, high=lidar_range, shape=(num_lidar_beams,), dtype=np.float32),
            spaces.Box(low=0, high=10, shape=(1,), dtype=np.float32) ,
            spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32) 
        ))

        # flag of new sensor info
        self._flag_all_received=False

        self._scan = LaserScan()
        self._robot_pose = Pose2D()
        self._robot_vel = Twist()
        self._subgoal =  Pose2D()
        
        # synchronization parameters
        self._first_sync_obs = True     # whether to return first sync'd obs or most recent
        self.max_deque_size = 10
        self._sync_slop = 0.05

        self._laser_deque = deque()
        self._rs_deque = deque()

        # message_filter subscriber: laserscan, robot_pose
        self._scan_sub = rospy.Subscriber(
            'scan', LaserScan, self.callback_scan, tcp_nodelay=True)

        self._robot_state_sub = rospy.Subscriber(
            'odom', Odometry, self.callback_robot_state, tcp_nodelay=True)
        
        # topic subscriber: subgoal
        self._subgoal_sub = rospy.Subscriber(
            'subgoal', PoseStamped, self.callback_subgoal)
        
        # service clients
        self._is_train_mode = rospy.get_param("train_mode")
        if self._is_train_mode:
            self._service_name_step='step_world'
            self._sim_step_client = rospy.ServiceProxy(self._service_name_step, StepWorld)

    
    def get_observation_space(self):
        return self.observation_space

    def get_observations(self):
        if self._is_train_mode:
            self.call_service_takeSimStep()
        else:
            try:
                rospy.wait_for_message(
                    "/next_cycle", Bool)
            except Exception:
                pass
        
        # try to retrieve sync'ed obs
        laser_scan, robot_pose = self.get_sync_obs()
        if laser_scan is not None and robot_pose is not None:
            # print("Synced successfully")
            self._scan = laser_scan
            self._robot_pose = robot_pose
        # else:
        #     print("Not synced")

        if len(self._scan.ranges) > 0:
            scan = self._scan.ranges.astype(np.float32)
        else:
            scan = np.zeros(360, dtype=float)

        rho, theta = ObservationCollector._get_goal_pose_in_robot_frame(self._subgoal, self._robot_pose)
        merged_obs = np.hstack([scan, np.array([rho,theta])])
        obs_dict = {}
        obs_dict["laser_scan"] = scan
        obs_dict['goal_in_robot_frame'] = [rho,theta]

        self._laser_deque.clear()
        self._rs_deque.clear()
        return merged_obs, obs_dict

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
    
    @staticmethod
    def _get_goal_pose_in_robot_frame(goal_pos:Pose2D,robot_pos:Pose2D):
         y_relative = goal_pos.y - robot_pos.y
         x_relative = goal_pos.x - robot_pos.x
         rho =  (x_relative**2+y_relative**2)**0.5
         theta = (np.arctan2(y_relative,x_relative)-robot_pos.theta+4*np.pi)%(2*np.pi)-np.pi
         return rho,theta

    def call_service_takeSimStep(self):
        request=StepWorldRequest()
        try:
            response=self._sim_step_client(request)
            rospy.logdebug("step service=",response)
        except rospy.ServiceException as e:
            rospy.logdebug("step Service call failed: %s"%e)

    def callback_scan(self, msg_laserscan):
        if len(self._laser_deque) == self.max_deque_size:
            self._laser_deque.popleft()
        self._laser_deque.append(msg_laserscan)
    
    def callback_robot_state(self, msg_robotstate):
        if len(self._rs_deque) == self.max_deque_size:
            self._rs_deque.popleft()
        self._rs_deque.append(msg_robotstate)

    def callback_subgoal(self,msg_Subgoal):
        self._subgoal=self.process_subgoal_msg(msg_Subgoal)
        return
        
    def callback_observation_received(self,msg_LaserScan,msg_RobotStateStamped):
        # process sensor msg
        self._scan=self.process_scan_msg(msg_LaserScan)
        self._robot_pose,self._robot_vel=self.process_robot_state_msg(msg_RobotStateStamped)
        # ask subgoal service
        #self._subgoal=self.call_service_askForSubgoal()
        self._flag_all_received=True
        
    def process_scan_msg(self, msg_LaserScan):
        # remove_nans_from_scan
        scan = np.array(msg_LaserScan.ranges)
        scan[np.isnan(scan)] = msg_LaserScan.range_max
        msg_LaserScan.ranges = scan
        return msg_LaserScan
    
    def process_robot_state_msg(self, msg_Odometry):
        pose3d = msg_Odometry.pose.pose
        twist = msg_Odometry.twist.twist
        return self.pose3D_to_pose2D(pose3d), twist
        
    def process_pose_msg(self,msg_PoseWithCovarianceStamped):
        # remove Covariance
        pose_with_cov=msg_PoseWithCovarianceStamped.pose
        pose=pose_with_cov.pose
        return self.pose3D_to_pose2D(pose)
    
    def process_subgoal_msg(self,msg_Subgoal):
        pose2d=self.pose3D_to_pose2D(msg_Subgoal.pose)
        return pose2d

    @staticmethod
    def pose3D_to_pose2D(pose3d):
        pose2d=Pose2D()
        pose2d.x=pose3d.position.x
        pose2d.y=pose3d.position.y
        quaternion=(pose3d.orientation.x,pose3d.orientation.y,pose3d.orientation.z,pose3d.orientation.w)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        pose2d.theta=yaw
        return pose2d
    @staticmethod
    def _stack_spaces(ss:Tuple[spaces.Box]):
        low = []
        high = []
        for space in ss:
            low.extend(space.low.tolist())
            high.extend(space.high.tolist())
        return spaces.Box(np.array(low).flatten(),np.array(high).flatten())


        
   

if __name__ == '__main__':
    
    rospy.init_node('states', anonymous=True)
    print("start")

    state_collector=ObservationCollector(360,10)
    i=0
    r=rospy.Rate(100)
    while(i<=1000):
        i=i+1
        obs=state_collector.get_observations()
        
        time.sleep(0.001)
        


    



