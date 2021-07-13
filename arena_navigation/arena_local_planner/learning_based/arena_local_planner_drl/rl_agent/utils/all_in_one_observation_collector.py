import time  # for debuging
from collections import deque
from typing import Tuple

import numpy as np
import rospy
# services
from flatland_msgs.srv import StepWorld, StepWorldRequest
from geometry_msgs.msg import Pose2D, PoseStamped
from geometry_msgs.msg import Twist
from gym import spaces
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock
# observation msgs
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
# for transformations
from tf.transformations import *


class ObservationCollectorAllInOne:
    def __init__(self, ns: str, num_lidar_beams: int, lidar_range: float, numb_models: int = 0,
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
                spaces.Box(low=0, high=10, shape=(1,), dtype=np.float32),
                spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32),
                spaces.Box(low=0, high=10, shape=(2,), dtype=np.float32),
                spaces.Box(low=-2.7, high=2.7, shape=(2*numb_models,), dtype=np.float32)
            ))
        else:
            self.observation_space = ObservationCollectorAllInOne._stack_spaces((
                spaces.Box(low=0, high=lidar_range, shape=(
                    num_lidar_beams,), dtype=np.float32),
                spaces.Box(low=0, high=10, shape=(1,), dtype=np.float32),
                spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32),
                spaces.Box(low=0, high=10, shape=(2,), dtype=np.float32)
            ))

        self._clock = Clock()
        self._scan = LaserScan()
        self._robot_pose = Pose2D()
        self._robot_vel = Twist()
        self._subgoal = Pose2D()
        self._globalplan = np.array([])
        self._twist = Twist()

        # train mode?
        self._is_train_mode = rospy.get_param("/train_mode")

        # synchronization parameters
        self._first_sync_obs = True  # whether to return first sync'd obs or most recent
        self.max_deque_size = 10
        self._sync_slop = 0.05

        self._laser_deque = deque()
        self._rs_deque = deque()

        # subscriptions
        self._scan_sub = rospy.Subscriber(
            f'{self.ns_prefix}scan', LaserScan, self.callback_scan, tcp_nodelay=True)

        self._robot_state_sub = rospy.Subscriber(
            f'{self.ns_prefix}odom', Odometry, self.callback_robot_state, tcp_nodelay=True)

        # self._clock_sub = rospy.Subscriber(
        #     f'{self.ns_prefix}clock', Clock, self.callback_clock, tcp_nodelay=True)

        self._subgoal_sub = rospy.Subscriber(
            f'{self.ns_prefix}subgoal', PoseStamped, self.callback_subgoal)

        self._globalplan_sub = rospy.Subscriber(
            f'{self.ns_prefix}globalPlan', Path, self.callback_global_plan)

        # Set publisher to publish filtered scan messages (relevant for costmap updates)
        # self._filtered_scan_pub = rospy.Publisher(f'{self.ns_prefix}filtered_scan', LaserScan)

        # service clients
        if self._is_train_mode:
            self._service_name_step = f'{self.ns_prefix}step_world'
            self._sim_step_client = rospy.ServiceProxy(
                self._service_name_step, StepWorld)

    def get_observation_space(self):
        return self.observation_space

    def get_observations(self):
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
        laser_scan, robot_pose, twist = self.get_sync_obs()

        if laser_scan is not None and robot_pose is not None and twist is not None:
            self._scan = laser_scan
            self._robot_pose = robot_pose
            self._twist = twist

        if len(self._scan.ranges) > 0:
            scan = self._scan.ranges.astype(np.float32)
        else:
            scan = np.zeros(self._laser_num_beams, dtype=float)

        rho, theta = ObservationCollectorAllInOne._get_goal_pose_in_robot_frame(
            self._subgoal, self._robot_pose)

        local_goal_x, local_goal_y = ObservationCollectorAllInOne._get_local_goal_in_robot_frame_xy(
            self._subgoal, self._robot_pose)

        if len(self._globalplan) == 0:
            global_goal = [0, 0]
        else:
            global_goal = self._globalplan[-1,:]

        merged_obs = np.float32(np.hstack([scan, np.array([rho, theta]), global_goal]))

        obs_dict = {'laser_scan': scan,
                    'goal_map_frame': self._subgoal,
                    'goal_in_robot_frame': [rho, theta],
                    'goal_in_robot_frame_xy': [local_goal_x, local_goal_y],
                    'global_plan': self._globalplan,
                    'robot_pose': self._robot_pose,
                    'robot_twist': self._twist,
                    'global_goal': global_goal
                    }

        self._laser_deque.clear()
        self._rs_deque.clear()
        return merged_obs, obs_dict

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

    def get_sync_obs(self):
        laser_scan = None
        robot_pose = None
        twist = None

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

        # print(f"Laser_stamp: {laser_stamp}, Robot_stamp: {robot_stamp}")
        return laser_scan, robot_pose, twist

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

    def callback_global_plan(self, msg_global_plan):
        self._globalplan = ObservationCollectorAllInOne.process_global_plan_msg(msg_global_plan)
        return

    def callback_scan(self, msg_laserscan: LaserScan):
        # republish filtered scan msg
        # scan_np = np.array(msg_laserscan.ranges)
        # scan_np[np.isnan(scan_np)] = 0 # msg_laserscan.range_max
        # msg_laserscan.ranges = scan_np
        # self._filtered_scan_pub.publish(msg_laserscan)

        # save message
        if len(self._laser_deque) == self.max_deque_size:
            self._laser_deque.popleft()
        self._laser_deque.append(msg_laserscan)

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
    def _stack_spaces(ss: Tuple[spaces.Box]):
        low = []
        high = []
        for space in ss:
            low.extend(space.low.tolist())
            high.extend(space.high.tolist())
        return spaces.Box(np.array(low).flatten(), np.array(high).flatten())
