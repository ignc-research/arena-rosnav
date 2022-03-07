#! /usr/bin/env python3
import numpy as np
from collections import deque
import rospy
import time
import math
import yaml

import time  # for debuging

# observation msgs
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import Path, Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest

# for transformations
from tf.transformations import *

from scipy.signal import lfilter, argrelextrema, filtfilt

class Hardcode:
    def __init__(
        self,
        ns: str="",
    ):
        self.ns = ns
        self.ns_prefix = "" if (ns == "" or ns is None) else "/" + ns + "/"

        self.have_goal_ = False
        self.have_odom_ = False

        self._sync_slop = 0.05
        self._first_sync_obs = (True)

        self.max_deque_size = 10
        self._laser_deque = deque()
        self._rs_deque = deque()
        self._scan = LaserScan()
        self._robot_pose = Pose2D()
        self._goal = Pose2D()
        self._subgoal = Pose2D()
        self._globalplan = np.array([])

        self.subgoal_DRL_timer_ = rospy.Timer(rospy.Duration(0.05), self.updateSubgoalDRLCallback)

        self._scan_sub = rospy.Subscriber(f"{self.ns_prefix}scan", LaserScan, self.callback_scan)
        self._robot_state_sub = rospy.Subscriber(f"{self.ns_prefix}odom", Odometry, self.callback_robot_state)
        self._goal_sub = rospy.Subscriber(f"{self.ns_prefix}goal", PoseStamped, self.callback_goal)
        
        self._globalPlan_pub = rospy.Publisher(f"{self.ns_prefix}globalPlan", Path, queue_size=10)
        self._subgoal_pub = rospy.Publisher(f"{self.ns_prefix}subgoal", PoseStamped, queue_size=10)

        with open('/home/baoduc/arena_ws/src/arena-rosnav/simulator_setup/robot/burger.model.yaml', "r") as fd:
            robot_data = yaml.safe_load(fd)

            for plugin in robot_data["plugins"]:
                if (
                    plugin["type"] == "Laser"
                    and plugin["name"] == "static_laser"
                ):
                    laser_angle_min = plugin["angle"]["min"]
                    laser_angle_max = plugin["angle"]["max"]
                    laser_angle_increment = plugin["angle"]["increment"]
                    self._laser_num_beams = int(
                        round(
                            (laser_angle_max - laser_angle_min)
                            / laser_angle_increment
                        )
                    )
                    self._laser_max_range = plugin["range"]

        self.planing_horizon = self._laser_max_range

        self.safety_zone = 3
        self.goal_tolerance = 0.7
        self.subgoal_tolerance = 0.1
        self.subgoal_togo = Pose2D()
        self.move_ref = 0
        self.alpha = 1
        self.limit = 4.0

    def updateSubgoalDRLCallback(self, event=None):
        if(not self.have_goal_):
            #print("not have a goal")
            return

        move_distance = 0
        laser_scan, robot_pose = self.get_sync_obs()
        if laser_scan is not None and robot_pose is not None:
            self._scan = laser_scan
            self._robot_pose = robot_pose
        try:
            if len(self._scan.ranges) > 0:
                scan = self._scan.ranges.astype(np.float32)
                #scan = scan[np.where(scan <= self.safety_zone, self.safety_zone - scan, 0)]
                angle = np.arange(self._scan.angle_min, self._scan.angle_max, self._scan.angle_increment)
                
                indies, dists_filtered = self.findLocalMaxima(self.planing_horizon - scan)
                
                numberOfMaxima = len(indies)

                passing = dists_filtered[indies]

                move_distance = self.alpha * np.sum(passing*np.sin(angle[indies])) / numberOfMaxima
                if move_distance > 0 :
                    move_distance = min(min(scan[(angle > -np.pi/2-self._scan.angle_increment) & 
                                                    (angle < -np.pi/2+self._scan.angle_increment)]), move_distance)
                else:
                    move_distance = -min(min(scan[(angle > -np.pi/2-self._scan.angle_increment) & 
                                                    (angle < -np.pi/2+self._scan.angle_increment)]), abs(move_distance))
        except:
            pass

        self.get_global_plan()
        dist_to_goal = self.get_distance(self._robot_pose, self._goal)
        if(dist_to_goal <= self.goal_tolerance):
            rospy.set_param("/bool_goal_reached", True)
            print("[Hardcode] Goal reached!", dist_to_goal)
            return

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"

        if dist_to_goal <= self.planing_horizon:
            pose_stamped.pose.position.x = self._goal.x
            pose_stamped.pose.position.y = self._goal.y
            self._subgoal_pub.publish(pose_stamped)
            return

        global_plan = self._globalplan  

        if (move_distance >= self.subgoal_tolerance) and (move_distance <= self.limit):
            self._subgoal.x = self._robot_pose.x + move_distance*np.sin(self._robot_pose.theta)
            self._subgoal.y = self._robot_pose.y - move_distance*np.cos(self._robot_pose.theta)
        else:
            self.move_ref = 0
            subgoal_id = 0
            for i in range(len(global_plan)):
                wp = Pose2D()
                wp.x = global_plan[i][0]
                wp.y = global_plan[i][1]
                dist_to_robot = self.get_distance(wp, self._robot_pose)
                if (dist_to_robot<self.planing_horizon+self.subgoal_tolerance) and (dist_to_robot>self.planing_horizon-self.subgoal_tolerance):
                    if i > subgoal_id:
                        subgoal_id = i
            if subgoal_id > 0:
                self._subgoal.x = global_plan[subgoal_id][0]
                self._subgoal.y = global_plan[subgoal_id][1]
            else:
                return

        pose_stamped.pose.position.x = self._subgoal.x
        pose_stamped.pose.position.y = self._subgoal.y
        self._subgoal_pub.publish(pose_stamped)

    def findLocalMaxima(self, data):
        n = 10  # the larger n is, the smoother curve will be
        b = [1.0 / n] * n
        a = 1
        data_filtered = np.insert(np.append(filtfilt(b,a,data),0),0,0)
        indies_maxima = np.array(argrelextrema(data_filtered, np.greater)[0])
        temp = indies_maxima[0]
        indies = []
        indies.append(temp)
        for i in range(len(indies_maxima)):
            if (indies_maxima[i] - temp < 72) and abs(data_filtered[indies_maxima[i]]-data_filtered[temp])<0.1:
                continue
            temp = indies_maxima[i]
            indies.append(indies_maxima[i])
        return np.array(indies)-1, data_filtered[1:-1]

    def callback_scan(self, msg_laserscan):
        if len(self._laser_deque) == self.max_deque_size:
            self._laser_deque.popleft()
        self._laser_deque.append(msg_laserscan) 

    def callback_robot_state(self, msg_robotstate):
        if len(self._rs_deque) == self.max_deque_size:
            self._rs_deque.popleft()
        self._rs_deque.append(msg_robotstate)
        self.have_odom_ = True

    def callback_goal(self, msg_Goal):
        if not self.have_odom_:
            print("not have an odom")
            return
        rospy.set_param("/bool_goal_reached", False)

        self._goal = self.process_goal_msg(msg_Goal)
        self.have_goal_= True
        print("[Hardcode] Goal set!")
        self.get_global_plan()
        return

    def get_global_plan(self):
        service_getPath = "/move_base/NavfnROS/make_plan" 
        rospy.wait_for_service(service_getPath)
        get_plan = rospy.ServiceProxy(service_getPath, GetPlan)
        if (not get_plan):
            rospy.logfatal("[Hardocde - GET_PATH] Could not initialize get plan service from %s", get_plan.getService().c_str())
        req = GetPlanRequest()
        
        req.start.header.frame_id ="map"
        req.start.pose.position.x = self._robot_pose.x
        req.start.pose.position.y = self._robot_pose.y
        req.start.pose.orientation.w = 1.0
        req.goal.header.frame_id = "map"
        req.goal.pose.position.x = self._goal.x
        req.goal.pose.position.y = self._goal.y
        req.goal.pose.orientation.w = 1.0
        req.tolerance = 0.7

        resp = get_plan(req)
        if resp.plan.poses != []:
            self._globalPlan_pub.publish(resp.plan)
            self._globalplan = Hardcode.process_global_plan_msg(resp.plan)
        else:
            print("could not find a global path")

    def get_sync_obs(self):
        laser_scan = None
        robot_pose = None

        while len(self._rs_deque) > 0 and len(self._laser_deque) > 0:
            laser_scan_msg = self._laser_deque.popleft()
            robot_pose_msg = self._rs_deque.popleft()

            laser_stamp = laser_scan_msg.header.stamp.to_sec()
            robot_stamp = robot_pose_msg.header.stamp.to_sec()

            while abs(laser_stamp - robot_stamp) > self._sync_slop:
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

        return laser_scan, robot_pose

    def process_scan_msg(self, msg_LaserScan: LaserScan):
        self._scan_stamp = msg_LaserScan.header.stamp.to_sec()
        scan = np.array(msg_LaserScan.ranges)
        scan[np.isnan(scan)] = msg_LaserScan.range_max
        msg_LaserScan.ranges = scan
        return msg_LaserScan
    
    def process_robot_state_msg(self, msg_Odometry):
        pose3d = msg_Odometry.pose.pose
        twist = msg_Odometry.twist.twist
        return self.pose3D_to_pose2D(pose3d), twist

    def process_goal_msg(self, msg_Goal):
        return self.pose3D_to_pose2D(msg_Goal.pose)

    @staticmethod
    def _get_goal_pose_in_robot_frame(goal_pos: Pose2D, robot_pos: Pose2D):
        y_relative = goal_pos.y - robot_pos.y
        x_relative = goal_pos.x - robot_pos.x
        rho = (x_relative ** 2 + y_relative ** 2) ** 0.5
        theta = (
            np.arctan2(y_relative, x_relative) - robot_pos.theta + 4 * np.pi
        ) % (2 * np.pi) - np.pi
        return rho, theta

    @staticmethod
    def process_global_plan_msg(globalplan):
        global_plan_2d = list(
            map(
                lambda p: Hardcode.pose3D_to_pose2D(p.pose),
                globalplan.poses,
            )
        )
        return np.array(list(map(lambda p2d: [p2d.x, p2d.y], global_plan_2d)))

    @staticmethod
    def pose3D_to_pose2D(pose3d):
        pose2d = Pose2D()
        pose2d.x = pose3d.position.x
        pose2d.y = pose3d.position.y
        quaternion = (pose3d.orientation.x, pose3d.orientation.y, pose3d.orientation.z, pose3d.orientation.w)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        pose2d.theta = yaw
        return pose2d

    @staticmethod
    def get_distance(pose_1: Pose2D, pose_2: Pose2D):
        return math.hypot(pose_2.x - pose_1.x, pose_2.y - pose_1.y)

if __name__ == "__main__":

    print("=== Hardcode started ===")
    rospy.init_node("hardcode")
    waypoint = Hardcode("")
    rospy.spin()