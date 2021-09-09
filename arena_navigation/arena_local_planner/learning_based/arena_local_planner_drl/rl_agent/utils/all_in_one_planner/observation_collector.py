import os
import subprocess
import time  # for debuging
from collections import deque

import nav_msgs
import numpy as np
import rospkg
import rospy
# services
import rosservice
import std_srvs.srv
import visualization_msgs.msg
from geometry_msgs.msg import Pose2D, PoseStamped, Pose
from geometry_msgs.msg import Twist
from gym import spaces
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
# observation msgs
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
# for transformations
from tf.transformations import *
from visualization_msgs.msg import Marker

import all_in_one_global_planner_interface.srv
from flatland_msgs.srv import StepWorld, StepWorldRequest


class ObservationCollectorAllInOne:
    def __init__(self, ns: str, num_lidar_beams: int, lidar_range: float, numb_models: int, required_obs: dict,
                 include_model_actions: bool = False):

        self.ns = ns
        if ns is None or ns == "":
            self.ns_prefix = "/"
        else:
            self.ns_prefix = "/" + ns + "/"

        self._laser_num_beams = rospy.get_param("/laser_num_beams")
        # for frequency controlling
        self._action_frequency = 1 / rospy.get_param("/robot_action_rate")

        # define observation_space
        if include_model_actions:
            self.observation_space = ObservationCollectorAllInOne._stack_spaces((
                spaces.Box(low=0, high=lidar_range, shape=(
                    num_lidar_beams,), dtype=np.float32),
                spaces.Box(low=0, high=lidar_range, shape=(
                    num_lidar_beams,), dtype=np.float32),
                spaces.Box(low=0, high=20, shape=(1,), dtype=np.float32),
                spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32),
                spaces.Box(low=0, high=20, shape=(1,), dtype=np.float32),
                spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32),
                spaces.Box(low=-2.7, high=2.7, shape=(2 * numb_models,), dtype=np.float32)
            ))
        else:
            self.observation_space = ObservationCollectorAllInOne._stack_spaces((
                spaces.Box(low=0, high=lidar_range, shape=(
                    num_lidar_beams,), dtype=np.float32),
                spaces.Box(low=0, high=lidar_range, shape=(
                    num_lidar_beams,), dtype=np.float32),
                spaces.Box(low=0, high=20, shape=(1,), dtype=np.float32),
                spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32),
                spaces.Box(low=0, high=20, shape=(1,), dtype=np.float32),
                spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32)
            ))

        self._required_obs = required_obs

        self._clock = Clock()
        self._scan = LaserScan()
        self._scan_static = LaserScan()
        self._robot_pose = Pose2D()
        self._robot_vel = Twist()
        self._goal = Pose2D()
        self._goal3D = Pose()
        self._subgoal = Pose2D()
        self._old_subgoal = Pose2D()
        self._globalplan = np.array([])
        self._globalplan_raw = nav_msgs.msg.Path()
        self._twist = Twist()
        self._new_global_plan = True

        # train mode?
        self._is_train_mode = rospy.get_param("/train_mode")

        # TODO make this a parameter
        if self._is_train_mode:
            self._planning_horizon = 4.5
        else:
            self._planning_horizon = 3.5

        # synchronization parameters
        self._first_sync_obs = True  # whether to return first sync'd obs or most recent
        self.max_deque_size = 10
        self._sync_slop = 0.05

        self._laser_deque = deque()
        self._laser_static_deque = deque()
        self._rs_deque = deque()

        self._dyn_scan_msg = None

        # subscriptions
        self._scan_sub = rospy.Subscriber(
            f'{self.ns_prefix}scan', LaserScan, self.callback_scan, tcp_nodelay=True)

        self._scan_static_sub = rospy.Subscriber(
            f'{self.ns_prefix}scan_static', LaserScan, self.callback_scan_static, tcp_nodelay=True)

        self._robot_state_sub = rospy.Subscriber(
            f'{self.ns_prefix}odom', Odometry, self.callback_robot_state, tcp_nodelay=True)

        # self._dynamic_scan_pub = rospy.Publisher(f'{self.ns_prefix}scan_dynamic', LaserScan, queue_size=1)

        # self._clock_sub = rospy.Subscriber(
        #     f'{self.ns_prefix}clock', Clock, self.callback_clock, tcp_nodelay=True)

        self._subgoal_visualizer = rospy.Publisher(f'{self.ns_prefix}vis_subgoal', visualization_msgs.msg.Marker,
                                                   queue_size=1)

        self._goal_sub = rospy.Subscriber(f'{self.ns_prefix}goal', PoseStamped, self.callback_goal)

        self._start_global_planner()

        if 'laser_3' in self._required_obs and self._required_obs['laser_3']:
            self._last_three_laser_scans = np.zeros((self._laser_num_beams, 3))
            self._needs_last_three_laser = True
        else:
            self._needs_last_three_laser = False

        # Set publisher to publish filtered scan messages (relevant for costmap updates)
        # self._filtered_scan_pub = rospy.Publisher(f'{self.ns_prefix}filtered_scan', LaserScan)

        # service clients
        if self._is_train_mode:
            self._service_name_step = f'{self.ns_prefix}step_world'
            self._sim_step_client = rospy.ServiceProxy(
                self._service_name_step, StepWorld)

    def get_observation_space(self):
        return self.observation_space

    def get_observations(self, make_new_global_plan: bool = False):
        # apply action time horizon
        if self._is_train_mode:
            self.call_service_takeSimStep(self._action_frequency)
        else:
            try:
                rospy.wait_for_message(
                    f"{self.ns_prefix}next_cycle", Bool)
            except Exception:
                pass

        # try to retrieve sync'ed obs
        laser_scan, laser_scan_static, robot_pose, twist = self.get_sync_obs()

        if laser_scan is not None and robot_pose is not None and twist is not None and laser_scan_static is not None:
            self._scan = laser_scan
            self._robot_pose = robot_pose
            self._twist = twist
            self._scan_static = laser_scan_static

        if len(self._scan.ranges) > 0 and len(self._scan_static.ranges) > 0:
            scan = self._scan.ranges.astype(np.float32)
            scan_static = self._scan_static.ranges.astype(np.float32)
            dynamic_scan = self.get_dynamic_scan(scan_static, scan)
        else:
            scan = np.zeros(self._laser_num_beams, dtype=float)
            scan_static = np.zeros(self._laser_num_beams, dtype=float)
            dynamic_scan = np.zeros(self._laser_num_beams, dtype=float)

        # create new global plan if necessary, extract subgoal and calculate distance to global plan
        if make_new_global_plan or self._global_plan_service is None:
            if self._global_plan_service is None:
                service_available = self._wait_for_global_plan_service()
            else:
                service_available = True
            if service_available:
                self._make_new_global_plan(self._robot_pose)
                self._new_global_plan = True
            # extract subgoal from new global plan
            self._subgoal = self._extract_subgoal(self._globalplan)
            self._visualize_subgoal(self._subgoal)

        rho_local, theta_local = ObservationCollectorAllInOne._get_goal_pose_in_robot_frame(
            self._subgoal, self._robot_pose)

        rho_global, theta_global = ObservationCollectorAllInOne._get_goal_pose_in_robot_frame(
            self._goal, self._robot_pose)

        local_goal_x, local_goal_y = ObservationCollectorAllInOne._get_local_goal_in_robot_frame_xy(
            self._subgoal, self._robot_pose)

        merged_obs = np.float32(
            np.hstack(
                [scan_static, dynamic_scan, np.array([rho_global, theta_global]), np.array([rho_local, theta_local])]))

        obs_dict = {'laser_scan': scan,
                    'scan_dynamic': dynamic_scan,
                    'goal_map_frame': self._subgoal,
                    'goal_in_robot_frame': [rho_local, theta_local],
                    'goal_in_robot_frame_xy': [local_goal_x, local_goal_y],
                    'global_plan': self._globalplan,
                    'new_global_plan': self._new_global_plan,
                    'global_plan_raw': self._globalplan_raw,
                    'robot_pose': self._robot_pose,
                    'robot_twist': self._twist,
                    'global_goal': np.array([self._goal.x, self._goal.y]),
                    'global_goal_robot_frame': np.array([rho_global, theta_global])
                    }

        # if necessary add last 3 laser scans to obs dict
        if self._needs_last_three_laser:
            # self._last_three_laser_scans[:, 1:2] = self._last_three_laser_scans[:, 0:1]
            # self._last_three_laser_scans[:, 0] = scan
            self._last_three_laser_scans[:, 0] = scan
            self._last_three_laser_scans[:, 1] = scan
            self._last_three_laser_scans[:, 2] = scan
            obs_dict['laser_3'] = self._last_three_laser_scans

        self._laser_deque.clear()
        self._laser_static_deque.clear()
        self._rs_deque.clear()

        self._new_global_plan = False

        return merged_obs, obs_dict

    def reset(self):
        if self._reset_global_costmap_service is not None:
            self._reset_global_costmap_service()

        if self._needs_last_three_laser:
            if len(self._scan.ranges) > 0:
                scan = self._scan.ranges.astype(np.float32)
            else:
                scan = np.zeros(self._laser_num_beams, dtype=float)
            self._last_three_laser_scans = np.array(
                [scan, scan, scan]).transpose()

    def close(self):
        self._global_planner_process.terminate()

    def get_dynamic_scan(self, static_scan: np.ndarray, scan: np.ndarray):
        diff_scan = np.abs(scan - static_scan)
        dyanmic_scan = np.where(diff_scan < 0.1, 3.5, scan)

        # self._dyn_scan_msg.ranges = dyanmic_scan
        # self._dyn_scan_msg.header.stamp = rospy.get_rostime()
        # self._dynamic_scan_pub.publish(self._dyn_scan_msg)

        return dyanmic_scan

    def get_sync_obs(self):
        laser_scan = None
        static_scan = None
        robot_pose = None
        twist = None
        laser_stamp = None

        # print(f"laser deque: {len(self._laser_deque)}, robot state deque: {len(self._rs_deque)}")
        while len(self._rs_deque) > 0 and len(self._laser_deque) > 0:
            laser_scan_msg = self._laser_deque.popleft()
            robot_pose_msg = self._rs_deque.popleft()

            laser_stamp = laser_scan_msg.header.stamp.to_sec()
            robot_stamp = robot_pose_msg.header.stamp.to_sec()

            while not abs(laser_stamp - robot_stamp) <= self._sync_slop:
                if laser_stamp > robot_stamp:
                    if len(self._rs_deque) == 0:
                        return laser_scan, robot_pose, twist
                    robot_pose_msg = self._rs_deque.popleft()
                    robot_stamp = robot_pose_msg.header.stamp.to_sec()
                else:
                    if len(self._laser_deque) == 0:
                        return laser_scan, robot_pose, twist
                    laser_scan_msg = self._laser_deque.popleft()
                    laser_stamp = laser_scan_msg.header.stamp.to_sec()

            laser_scan = self.process_scan_msg(laser_scan_msg)
            robot_pose, twist = self.process_robot_state_msg(robot_pose_msg)

            if self._first_sync_obs:
                break

        # Extract static scan
        static_laser_stamp = 0
        static_laser_scan_msg = None
        if laser_stamp is not None:
            while laser_stamp > static_laser_stamp and len(self._laser_static_deque) > 0:
                static_laser_scan_msg = self._laser_static_deque.popleft()
                static_laser_stamp = static_laser_scan_msg.header.stamp.to_sec()

            if static_laser_scan_msg is not None:
                static_scan = self.process_scan_msg(static_laser_scan_msg)
                self._dyn_scan_msg = static_laser_scan_msg

        # print(f"Laser_stamp: {laser_stamp}, Static_stamp: {static_laser_stamp}")

        return laser_scan, static_scan, robot_pose, twist

    def call_service_takeSimStep(self, t=None):
        if t is None:
            request = StepWorldRequest()
        else:
            request = StepWorldRequest(t)
        timeout = 12
        try:
            for i in range(timeout):
                response = self._sim_step_client(request)
                rospy.logdebug("step service=", response)

                if response.success:
                    break
                if i == timeout - 1:
                    raise TimeoutError(
                        f"Timeout while trying to call '{self.ns_prefix}step_world'")
                time.sleep(0.33)

        except rospy.ServiceException as e:
            rospy.logdebug("step Service call failed: %s" % e)

    def callback_clock(self, msg_Clock):
        self._clock = msg_Clock.clock.to_sec()
        return

    def callback_subgoal(self, msg_Subgoal):
        self._subgoal = self.process_subgoal_msg(msg_Subgoal)
        return

    def callback_goal(self, msg_Goal):
        self._goal = self.pose3D_to_pose2D(msg_Goal.pose)
        self._goal3D = msg_Goal.pose
        return

    def callback_global_plan(self, msg_global_plan: nav_msgs.msg.Path):
        self._globalplan = ObservationCollectorAllInOne.process_global_plan_msg(msg_global_plan)
        self._globalplan_raw = msg_global_plan

        # change frame_id from /map to map
        for pose in self._globalplan_raw.poses:
            pose.header.frame_id = "map"

        self._new_global_plan = True

        return

    def callback_scan(self, msg_laserscan: LaserScan):
        # save message
        if len(self._laser_deque) == self.max_deque_size:
            self._laser_deque.popleft()
        self._laser_deque.append(msg_laserscan)

    def callback_scan_static(self, msg_laserscan: LaserScan):
        # save message
        if len(self._laser_static_deque) == self.max_deque_size:
            self._laser_static_deque.popleft()
        self._laser_static_deque.append(msg_laserscan)

    def callback_robot_state(self, msg_robotstate):
        if len(self._rs_deque) == self.max_deque_size:
            self._rs_deque.popleft()
        self._rs_deque.append(msg_robotstate)

    def callback_observation_received(self, msg_LaserScan, msg_RobotStateStamped):
        # process sensor msg
        self._scan = self.process_scan_msg(msg_LaserScan)
        self._robot_pose, self._robot_vel = self.process_robot_state_msg(msg_RobotStateStamped)
        self.obs_received = True
        return

    def process_scan_msg(self, msg_LaserScan: LaserScan):
        # remove_nans_from_scan
        self._scan_stamp = msg_LaserScan.header.stamp.to_sec()
        scan = np.array(msg_LaserScan.ranges)
        scan[np.isnan(scan)] = msg_LaserScan.range_max
        scan[scan > msg_LaserScan.range_max] = msg_LaserScan.range_max
        msg_LaserScan.ranges = scan
        return msg_LaserScan

    def process_robot_state_msg(self, msg_Odometry):
        pose3d = msg_Odometry.pose.pose
        twist = msg_Odometry.twist.twist
        return self.pose3D_to_pose2D(pose3d), twist

    def process_pose_msg(self, msg_PoseWithCovarianceStamped):
        # remove Covariance
        pose_with_cov = msg_PoseWithCovarianceStamped.pose
        pose = pose_with_cov.pose
        return self.pose3D_to_pose2D(pose)

    def process_subgoal_msg(self, msg_Subgoal):
        pose2d = self.pose3D_to_pose2D(msg_Subgoal.pose)
        return pose2d

    def _start_global_planner(self):
        # Generate local planner node
        config_path = os.path.join(rospkg.RosPack().get_path('arena_local_planner_drl'), 'configs',
                                   'all_in_one_hyperparameters', 'base_global_planner_parameters',
                                   'global_planner.yaml')
        package = 'all_in_one_global_planner_interface'
        launch_file = 'start_global_planner_node.launch'
        if self.ns == "":
            arg1 = "ns:=_"
            arg4 = "use_ns:=false"
        else:
            arg1 = "ns:=" + self.ns
            arg4 = "use_ns:=true"

        arg2 = "node_name:=" + 'global_planner'
        arg3 = "config_path:=" + config_path

        # Use subprocess to execute .launch file
        if self._is_train_mode:
            self._global_planner_process = subprocess.Popen(["roslaunch", package, launch_file, arg1, arg2, arg3, arg4],
                                                            stdout=subprocess.DEVNULL,
                                                            stderr=subprocess.STDOUT)
        else:
            self._global_planner_process = subprocess.Popen(["roslaunch", package, launch_file, arg1, arg2, arg3, arg4],
                                                            stdout=subprocess.DEVNULL)

        self._global_plan_service = None
        self._reset_global_costmap_service = None

    def _wait_for_global_plan_service(self) -> bool:
        # wait until service is available
        make_plan_service_name = self.ns_prefix + "global_planner" + "/" + "makeGlobalPlan"
        reset_costmap_service_name = self.ns_prefix + "global_planner" + "/" + "resetGlobalCostmap"
        service_list = rosservice.get_service_list()
        max_tries = 10
        for i in range(max_tries):
            if make_plan_service_name in service_list and reset_costmap_service_name in service_list:
                break
            else:
                time.sleep(0.3)

        if make_plan_service_name in service_list and reset_costmap_service_name in service_list:
            self._global_plan_service = rospy.ServiceProxy(make_plan_service_name,
                                                           all_in_one_global_planner_interface.srv.MakeNewPlan,
                                                           persistent=True)
            self._reset_global_costmap_service = rospy.ServiceProxy(reset_costmap_service_name,
                                                                    std_srvs.srv.Empty,
                                                                    persistent=True)
            return True
        else:
            return False

    def _make_new_global_plan(self, robot_pose):
        goal_msg = PoseStamped()
        goal_msg.pose = self._goal3D
        goal_msg.header.frame_id = "map"
        globalplan_raw = self._global_plan_service(goal_msg).global_plan

        if not len(globalplan_raw.poses) == 0:
            self._globalplan_raw = globalplan_raw
            self._globalplan = self.process_global_plan_msg(self._globalplan_raw)

            # change frame_id from /map to map
            for pose in self._globalplan_raw.poses:
                pose.header.frame_id = "map"

    def _visualize_subgoal(self, subgoal: Pose2D):
        subgoal_msg = Marker()
        subgoal_msg.header.frame_id = "map"
        subgoal_msg.id = 105
        subgoal_msg.action = Marker.ADD
        subgoal_msg.type = Marker.SPHERE
        subgoal_msg.color.r = 1
        subgoal_msg.color.g = 1
        subgoal_msg.color.b = 0
        subgoal_msg.color.a = 0.7
        subgoal_msg.scale.x = 0.7
        subgoal_msg.scale.y = 0.7
        subgoal_msg.scale.z = 0.01

        subgoal_msg.pose.position.x = subgoal.x
        subgoal_msg.pose.position.y = subgoal.y

        self._subgoal_visualizer.publish(subgoal_msg)

    def _extract_subgoal(self, global_plan) -> Pose2D:
        # extract subgoal by getting point on global path with @_planning_horizon distance

        if global_plan.size == 0:
            return self._goal

        start_xy = global_plan[0]
        end_xy = global_plan[-1]

        if global_plan.size <= 2:
            return Pose2D(end_xy[0], end_xy[1], 0)

        if np.linalg.norm(start_xy - end_xy) < self._planning_horizon:
            return Pose2D(end_xy[0], end_xy[1], 0)

        i = 1
        next_pose_xy = global_plan[i]
        while np.linalg.norm(start_xy - next_pose_xy) < self._planning_horizon and i < global_plan.size - 1:
            i += 1
            next_pose_xy = global_plan[i]

        return Pose2D(next_pose_xy[0], next_pose_xy[1], 0)

    @staticmethod
    def process_global_plan_msg(globalplan):
        global_plan_2d = list(map(
            lambda p: ObservationCollectorAllInOne.pose3D_to_pose2D(p.pose), globalplan.poses))
        global_plan_np = np.array(list(map(lambda p2d: [p2d.x, p2d.y], global_plan_2d)))
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
    def _get_local_goal_in_robot_frame_xy(goal_pos: Pose2D, robot_pos: Pose2D):
        [x, y, theta] = [robot_pos.x, robot_pos.y, robot_pos.theta]  # self position based on map
        [goal_x, goal_y] = [goal_pos.x, goal_pos.y]  # sub goal based on map
        local_x = (goal_x - x) * np.cos(theta) + (goal_y - y) * np.sin(theta)
        local_y = -(goal_x - x) * np.sin(theta) + (goal_y - y) * np.cos(theta)
        return [local_x, local_y]  # return subgoal position based on robot

    @staticmethod
    def _get_goal_pose_in_robot_frame(goal_pos: Pose2D, robot_pos: Pose2D):
        y_relative = goal_pos.y - robot_pos.y
        x_relative = goal_pos.x - robot_pos.x
        rho = (x_relative ** 2 + y_relative ** 2) ** 0.5
        theta = (np.arctan2(y_relative, x_relative) -
                 robot_pos.theta + 4 * np.pi) % (2 * np.pi) - np.pi
        return rho, theta

    @staticmethod
    def _stack_spaces(ss):
        low = []
        high = []
        for space in ss:
            low.extend(space.low.tolist())
            high.extend(space.high.tolist())
        return spaces.Box(np.array(low).flatten(), np.array(high).flatten())
