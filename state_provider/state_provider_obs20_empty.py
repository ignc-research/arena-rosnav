#!/usr/bin/env python
# ros
import rospy
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Pose2D
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
# viz
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

import math
from torch.nn.utils.rnn import pack_sequence
import torch
import numpy as np

# for transformations
from tf.transformations import *

class StateProvider():
    """A class that provides preprocessed state information for everyone to access.
    Currently, following information are provided 
        - robot position [PoseStamped]
        - goal position  [PoseStamped]
        - distance robot to goal [Float]
        - distance robot to subgoal
        - global plan [Path]
    """
    def __init__(self):

        #
        self.i = 0 
        self.j = 1
        self._globalGoal = PoseStamped()
        self._globalPlan = Path()
        self._subgoal = PoseStamped()
        self._robot_pose = PoseStamped()
        self.navgoal = PoseStamped()
        self._globalplan = np.array([])

        #provide distance and angle to goal inside pose2D messages (x for distance, theta for the angle)
        self.distance2GlobalGoal = Pose2D()
        self.distance2subgoal = Pose2D()
        self.list_of_lists = [[0.17, 10.59], [4.96, 8.51], [10.79, 6.25], [15.34, 4,3], [19.52, 2.79], [23, 1]]
        self.wps = []

        self.navgoal_pub = rospy.Publisher('/subgoal_wpg', PoseStamped, queue_size=1)

        # subs
        self._robot_state_sub = rospy.Subscriber('/odom', Odometry, self.cbRobotPosition)
        self._globalGoal_sub = rospy.Subscriber('/goal', PoseStamped, self.cbGlobalGoal)
        self._globalPlan_sub = rospy.Subscriber('/plan_manager/globalPlan', Path, self.cbglobalPlan)   #todo in plan_manager.cpp publish the global plan
        self.subGoal_sub = rospy.Subscriber('/subgoal',PoseStamped, self.cbSubGoal)
        
        # pubs
        
        self.globalPlan_pub = rospy.Publisher('/state_provider/globalPlan', Path, queue_size=1)
        self.globalGoal_pub = rospy.Publisher('/state_provider/globalGoal', PoseStamped, queue_size=1)
        self.subgoal_pub = rospy.Publisher('/state_provider/subgoal', PoseStamped, queue_size=1)
        self.waypoints_pub = rospy.Publisher('/zzwp', Pose2D, queue_size=1)

        self.distance2global_pub = rospy.Publisher('/state_provider/distance2global', Pose2D, queue_size=1)
        self.distance2sub_pub = rospy.Publisher('/state_provider/distance2sub', Pose2D, queue_size=1)

        self.wps = None
        self.wp_pose2d = None
        self.wp_idx = 0

        self._num_wps = 7
        self._dist_to_wp = 1


        

    def cbRobotPosition(self,msg):
        self._robot_pose.pose.position.x = msg.pose.pose.position.x
        self._robot_pose.pose.position.y = msg.pose.pose.position.y
        self._robot_pose.pose.orientation = msg.pose.pose.orientation
        #self.navgoal_pub.publish(self._robot_pose)
        self.distance2GlobalGoal.x = self.provideDistance2Goal(self._globalGoal)[0]              #distance to goal
        self.distance2GlobalGoal.theta = self.provideDistance2Goal(self._globalGoal)[1]          #angle to goal
        self.distance2global_pub.publish(self.distance2GlobalGoal)         
        self.distance2subgoal.x = self.provideDistance2Goal(self.navgoal)[0]                 #distance to goal            #angle to goal
        self.distance2sub_pub.publish(self.distance2subgoal)  

        if self.distance2GlobalGoal.x < 0.8:
            self.j = 1
            self.i = 0

        # i 20 because ros has a delay in publishing the messages
        if self.i < 20: 
            self.navgoal.header.stamp = rospy.Time.now()
            self.navgoal.header.frame_id = "map"
            self.navgoal.pose.orientation.w = 1
            self.navgoal.pose.position.x = self.list_of_lists[0][0]
            self.navgoal.pose.position.y = self.list_of_lists[0][1]
            self.navgoal_pub.publish(self.navgoal)
            print(self.navgoal)
            self.i += 1

   
       

        if self.distance2subgoal.x < 0.9:
            self.navgoal.header.stamp = rospy.Time.now()
            self.navgoal.header.frame_id = "map"
            self.navgoal.pose.orientation.w = 1
            self.navgoal.pose.position.x = self.list_of_lists[self.j][0]
            self.navgoal.pose.position.y = self.list_of_lists[self.j][1]
            self.navgoal_pub.publish(self.navgoal)
            print("test2")
            self.j += 1



    def cbSubGoal(self,msg):
        self._subgoal = msg
        
        

    def cbGlobalGoal(self,msg):
        self._globalGoal = msg
        self.distance2GlobalGoal.x = self.provideDistance2Goal(msg)[0]              #distance to goal
        self.distance2GlobalGoal.theta = self.provideDistance2Goal(msg)[1]          #angle to goal
        self.distance2global_pub.publish(self.distance2GlobalGoal)                  #publish distance to topic /state_provider/distance2global
        self.globalGoal_pub.publish(self._globalGoal)


    def cbglobalPlan(self,msg):
        self._globalPlan = msg
        self.globalPlan_pub.publish(self._globalPlan)

        # self._globalplan = StateProvider.process_global_plan_msg(msg)
        # self._get_waypoints()
        # self.waypoints_pub.publish(self.wp_pose2d)

        # return
 


    
    def provideDistance2Goal(self, goal):
        robotdistance2d = self.pose3D_to_pose2D(self._robot_pose.pose)
        goal = self.pose3D_to_pose2D(goal.pose)
        return self._calc_distance(goal,robotdistance2d)

    def _get_waypoints(self):
        self.wp_idx = 0
        if self._num_wps > 1:
            idx = round(len(self._globalplan)/self._num_wps) # num of waypoints
            wps = self._globalplan[0::idx]
            self.wps = np.append(
                wps[:-1], [[self.navgoal.pose.position.x, self.navgoal.pose.position.y]], axis=0)
        else:
            self.wps = np.array([[self.navgoal.pose.position.x, self.navgoal.pose.position.y]])
        self._wp_to_posed2D(0)
        #print("drawn new waypoints!")  
        # 
    def _wp_to_posed2D(self, index):
        wp = self.wps[index]

        self.wp_pose2d = Pose2D()
        self.wp_pose2d.x = wp[0]
        self.wp_pose2d.y = wp[1]  
        
    @staticmethod
    def _calc_distance(goal_pos:Pose2D,robot_pos:Pose2D):
         y_relative = goal_pos.y - robot_pos.y
         x_relative = goal_pos.x - robot_pos.x
        #  d=np.array[x_relative,y_relative]
        #  dist = np.linalg.norm(d)
         rho =  (x_relative**2+y_relative**2)**0.5
         theta = (np.arctan2(y_relative,x_relative)-robot_pos.theta+4*np.pi)%(2*np.pi)-np.pi
         return rho,theta

    @staticmethod #static methods so can be used without instantiating this class, helpful for others
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
    def process_global_plan_msg(globalplan):
        global_plan_2d = list(map(
            lambda p: StateProvider.pose3D_to_pose2D(p.pose), globalplan.poses))
        global_plan_np = np.array(list(map(lambda p2d: [p2d.x,p2d.y], global_plan_2d)))
        
        # import pickle
        # with open('globalpath.pickle', 'wb') as handle:
        #     pickle.dump(global_plan_np, handle, protocol=pickle.HIGHEST_PROTOCOL)

        return global_plan_np


    def goalReached(self, distance:Pose2D):
        # how far away from goal?
        if distance.x > 0.3:
            return False
        else:
            return True
    

def run():

    rospy.init_node('stateprovider',anonymous=False)

    print('==================================\nStateProvider Node Started\n==================================')

    state_provider = StateProvider()

    rospy.spin()

if __name__ == '__main__':
    run()