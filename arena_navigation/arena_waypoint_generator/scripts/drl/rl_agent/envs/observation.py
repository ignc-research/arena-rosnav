#! /usr/bin/env python3
import numpy as np
from collections import deque
import rospy
from gym import spaces
from gym.spaces import space
import yaml
from typing import Tuple
from pathlib import Path
import time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.srv import GetPlan, GetPlanRequest

from tf.transformations import *

class Observation:
    def __init__(
        self,
        PATHS: dict = dict(),
        ns: str="",
    ):
        self.ns = ns
        self.ns_prefix = "" if (ns == "" or ns is None) else "/" + ns + "/"

        self._sync_slop = 0.05
        self._first_sync_obs = (True)

        self.max_deque_size = 10
        self._laser_deque = deque()
        self._rs_deque = deque()
        self._scan = LaserScan()
        self._robot_pose = Pose2D()
        self._goal = Pose2D()
        self._globalplan = np.array([])

        self._scan_sub = rospy.Subscriber(f"{self.ns_prefix}scan", LaserScan, self.callback_scan, tcp_nodelay=True)
        self._robot_state_sub = rospy.Subscriber(f"{self.ns_prefix}odom", Odometry, self.callback_robot_state, tcp_nodelay=True)
        self._goal_sub = rospy.Subscriber(f"{self.ns_prefix}goal", PoseStamped, self.callback_goal)
        
        self._globalPlan_pub = rospy.Publisher(f"{self.ns_prefix}globalPlan", Path, queue_size=10)
        
        with open(PATHS["robot_setting"], "r") as fd:
            robot_data = yaml.safe_load(fd)

            # get laser related information
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


        # observation parameters
        lidar_range = self._laser_max_range
        num_lidar_beams = self._laser_num_beams
        self.observation_space = Observation._stack_spaces((spaces.Box(low=0, high=lidar_range, shape=(num_lidar_beams,), dtype=np.float32,),
                                                            spaces.Box(low=0, high=50, shape=(1,), dtype=np.float32),))
                                                            #spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32)))
        self.global_plan_length = 0

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
            self._globalplan = Observation.process_global_plan_msg(resp.plan)
            return self._globalplan
        else:
            pass

    def callback_scan(self, msg_laserscan):
        if len(self._laser_deque) == self.max_deque_size:
            self._laser_deque.popleft()
        self._laser_deque.append(msg_laserscan) 

    def callback_robot_state(self, msg_robotstate):
        if len(self._rs_deque) == self.max_deque_size:
            self._rs_deque.popleft()
        self._rs_deque.append(msg_robotstate)

    def callback_goal(self, msg_Goal):
        self._goal = self.process_goal_msg(msg_Goal)
        return

    def get_observation_space(self):
        return self.observation_space

    def get_observations(self, *args, **kwargs):
        laser_scan, robot_pose = self.get_sync_obs()
        if laser_scan is not None and robot_pose is not None:
            self._scan = laser_scan
            self._robot_pose = robot_pose

        if len(self._scan.ranges) > 0:
            scan = self._scan.ranges.astype(np.float32)
        else:
            scan = np.zeros(self._laser_num_beams, dtype=np.float32)

        rho, theta = Observation._get_goal_pose_in_robot_frame(self._goal, self._robot_pose)

        scan_angle = np.array([self._scan.angle_min, self._scan.angle_max, self._scan.angle_increment])

        global_plan = self.get_global_plan()
        try:
            global_plan_length = np.hypot(*np.diff(global_plan.T)).sum()
        except:
            global_plan_length = 0

        #obs = (np.hstack([scan, np.array([rho, theta]).astype(np.float32)]))
        obs = (np.hstack([scan, np.array(global_plan_length).astype(np.float32)]))

        self.global_plan_length = global_plan_length

        obs_dict = {
            "laser_scan": scan,
            "goal_in_robot_frame": [rho, theta],
            "global_plan": self.get_global_plan(),
            "robot_pose": self._robot_pose,
            "goal_pose": self._goal,
            "scan_angle": scan_angle,
            "global_plan_length": global_plan_length
        }

        self._laser_deque.clear()
        self._rs_deque.clear()
        return obs, obs_dict

    def get_lidar_range(self):
        return self._laser_max_range

    def get_goal_pose(self):
        return self._goal

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
                lambda p: Observation.pose3D_to_pose2D(p.pose),
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
    def _stack_spaces(ss: Tuple[spaces.Box]):
        low = []
        high = []
        for space in ss:
            low.extend(space.low.tolist())
            high.extend(space.high.tolist())
        return spaces.Box(np.array(low).flatten(), np.array(high).flatten())
