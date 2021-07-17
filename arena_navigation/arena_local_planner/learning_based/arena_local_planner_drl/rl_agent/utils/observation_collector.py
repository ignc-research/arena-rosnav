#! /usr/bin/env python
import enum
import threading
from time import time
from typing import List, Optional, Tuple, Union

from enum import Enum, auto
import threading
from numpy.core.numeric import normalize_axis_tuple
import rospy
import random
import numpy as np
from collections import deque

import time  # for debuging
import threading
from rospy.core import rospydebug
# observation msgs
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty


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
    def __init__(self, ns: str, num_lidar_beams: int, lidar_range: float):
        """ a class to collect and merge observations

        Args:
            num_lidar_beams (int): [description]
            lidar_range (float): [description]
        """
        self.ns = ns
        if ns is None or ns == "":
            self.ns_prefix = ""
        else:
            self.ns_prefix = "/"+ns+"/"

        # define observation_space
        # subgoal global goal
        self.observation_space = ObservationCollector._stack_spaces((
            spaces.Box(low=0, high=lidar_range, shape=(
                num_lidar_beams,), dtype=np.float32),
            spaces.Box(low=0, high=10, shape=(1,), dtype=np.float32),
            spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32),

        ))

        self._laser_num_beams = rospy.get_param("/laser_num_beams")
        # for frequency controlling, this value should be set to a value same as laser_update_rate
        self._action_frequency = 1/rospy.get_param("/robot_action_rate")

        self._clock = Clock()
        self._scan = LaserScan()
        self._robot_pose = Pose2D()
        self._robot_vel = Twist()
        self._subgoal = Pose2D()
        self._globalGoal = PoseStamped()
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

        self.sub_global_goal = rospy.Subscriber(
            f'{self.ns_prefix}goal', PoseStamped, self.callback_globalGoal)

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
            scan = np.zeros(self._laser_num_beams, dtype=float)

        rho_goal, theta_goal = ObservationCollector._get_goal_pose_in_robot_frame(
            self._globalGoal, self._subgoal)
        rho, theta = ObservationCollector._get_goal_pose_in_robot_frame(
            self._subgoal, self._robot_pose)
        merged_obs = np.hstack([scan, np.array([rho, theta])])

        obs_dict = {}
        obs_dict['laser_scan'] = scan
        obs_dict['goal_in_robot_frame'] = [rho, theta]
        obs_dict['global_plan'] = self._globalplan
        obs_dict['robot_pose'] = self._robot_pose
        obs_dict['global_in_subgoal_frame'] = [rho_goal, theta_goal]

        self._laser_deque.clear()
        self._rs_deque.clear()
        return merged_obs, obs_dict

    @staticmethod
    def _get_goal_pose_in_robot_frame(goal_pos: Pose2D, robot_pos: Pose2D):
        y_relative = goal_pos.y - robot_pos.y
        x_relative = goal_pos.x - robot_pos.x
        rho = (x_relative**2+y_relative**2)**0.5
        theta = (np.arctan2(y_relative, x_relative) -
                 robot_pos.theta+4*np.pi) % (2*np.pi)-np.pi
        return rho, theta

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
        timeout = 12
        try:
            for i in range(timeout):
                response = self._sim_step_client(request)
                rospy.logdebug("step service=", response)

                if response.success:
                    break
                if i == timeout-1:
                    raise TimeoutError(
                        f"Timeout while trying to call '{self.ns_prefix}step_world'")
                time.sleep(0.33)

        except rospy.ServiceException as e:
            rospy.logdebug("step Service call failed: %s" % e)

    def callback_globalGoal(self, msg_Subgoal):
        self._globalGoal = self.process_subgoal_msg(msg_Subgoal)
        return

    def callback_clock(self, msg_Clock):
        self._clock = msg_Clock.clock.to_sec()
        return

    def callback_subgoal(self, msg_Subgoal):
        self._subgoal = self.process_subgoal_msg(msg_Subgoal)
        return

    def callback_global_plan(self, msg_global_plan):
        self._globalplan = ObservationCollector.process_global_plan_msg(
            msg_global_plan)
        return

    def callback_scan(self, msg_laserscan):
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
        self._robot_pose, self._robot_vel = self.process_robot_state_msg(
            msg_RobotStateStamped)
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
            lambda p: ObservationCollector.pose3D_to_pose2D(p.pose), globalplan.poses))
        global_plan_np = np.array(
            list(map(lambda p2d: [p2d.x, p2d.y], global_plan_2d)))
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


class ObservationCollectorWP():
    class Event(Enum):
        NEWLASER = auto()
        NEWROBOTSTATE = auto()
        COLLISIONDETECTED = auto()
        WAYPOINTARRIVAL = auto()
        GOALARRIVAL = auto()
        TIMEOUT = auto()

    def __init__(self, ns: str,
                 is_train_mode:bool,
                 num_lidar_beams: int,
                 lidar_angle_increment: float,
                 lidar_range: float,
                 robot_waypoint_min_dist: float,
                 robot_obstacle_min_dist: float
                 ):
        """ 
        A Observation collector made for way point generator.
        it supplies follwing functionalities
            1. laser scan, robot state , subgoal collection,
            2. enable the simulater flatland running simutaniously with a timeout.
            3. register rules to early stopping the simulator, e.g. the crash happened or
                the position of robot close to the set subgoal

        Args:
            ns(str): namespace of the training environment
            num_lidar_beams (int): [description]
            lidar_range (float): maximum length of the laser beam
            robot_waypoint_min_dist (float): the minimum distance between the robot and the waypoint,
                if actual distance is under this value, a stop-running signal will be send the simulator
            robot_obstacle_min_dist (float): the minimum distance between the robot and the surrunding obstacle
                if actual distance is under this value, a collision event will be registered.
        """
        self.ns = ns
        if ns is None or ns == "":
            self.ns_prefix = ""
        else:
            self.ns_prefix = "/"+ns+"/"
        self.is_train_mode= is_train_mode

        # define observation_space
        self.observation_space = ObservationCollectorWP._stack_spaces((
            spaces.Box(low=0, high=lidar_range, shape=(
                num_lidar_beams,), dtype=np.float32),
            spaces.Box(low=0, high=10, shape=(1,), dtype=np.float32),
            spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32),
            spaces.Box(low=0, high=10, shape=(1,), dtype=np.float32),
            spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32)
        ))
        self._num_lidar_beams = num_lidar_beams
        self._lidar_angle_increment = lidar_angle_increment
        self.robot_waypoint_min_dist = robot_waypoint_min_dist
        self._robot_obstacle_min_dist = robot_obstacle_min_dist

        # subscriptions
        # for update the laserscan after the reset of environment.
        self._robot_action_rate = rospy.get_param("/robot_action_rate")

        self._laserscan_sub = rospy.Subscriber(
            f'{self.ns_prefix}scan', LaserScan, self._callback_laserscan, tcp_nodelay=True)
        self._robot_state_sub = rospy.Subscriber(
            f'{self.ns_prefix}odom', Odometry, self._callback_robot_state, tcp_nodelay=True)
        self._subgoal_sub = rospy.Subscriber(
            f'{self.ns_prefix}subgoal', PoseStamped, self._callback_subgoal, tcp_nodelay=True)
        self._global_goal_sub = rospy.Subscriber(
            f'{self.ns_prefix}goal', PoseStamped, self._callback_globalgoal, tcp_nodelay=True)
        self._globalplan_sub = rospy.Subscriber(
            f'{self.ns_prefix}globalPlan', Path, self._callback_globalplan)

        if self.is_train_mode:
            self._step_world_srv = rospy.ServiceProxy(
                f'{self.ns_prefix}step_world', StepWorld)
            self._stop_step_world_pub = rospy.Publisher(
                f'{self.ns_prefix}stop_step_world', Empty, tcp_nodelay=True, queue_size=1)
        else:
            self.cv_important_event_deployment = threading.Condition()
        # define variables
        self._globalgoal: Optional[Pose2D] = None
        self._subgoal: Optional[Pose2D] = None
        self._old_subgoal: Optional[Pose2D] = None
        self._gloablplan = None
        self._laser_scans: List[np.ndarray] = []
        self._robot_states: List[Tuple[Pose2D, Twist]] = []
        self._waypoint: Optional[Pose2D] = None
        self.important_event: Optional[ObservationCollectorWP.Event] = None

    def wait_for_step_end(self, timeout: float = 1000000) -> 'ObservationCollectorWP.Event':
        """[summary]

        Args:
            timeout (float): maximum simulation time allow the local planner to execute.
        """

        request = StepWorldRequest(timeout)
        # run the simulator, collect observation and do the check.
        response = self._step_world_srv(request)
        rospy.logdebug("step service=", response)
        # two result
        #    1. timeout (fail)
        #    2.  collision or arrived at the set waypoint (success)
        # TODO log it save to rosbag ?
        if not response.success:
            self.important_event = ObservationCollectorWP.Event.TIMEOUT
        return self.important_event

    def wait_for_new_event(self,timeout:100):
        """used in deployment mode, the timeout here is real time, take care!
        """
        with self.cv_important_event_deployment:
            if not self.cv_important_event_deployment.wait(timeout):
                # time out 
                self.important_event = ObservationCollectorWP.Event.TIMEOUT
            # else it coule be reach the goal
        return self.important_event
    

    def get_laserscans(self, num_laserscans: int, convert_all_on_latest_robot_frame: bool) -> np.ndarray:
        """get the laser scans and robot's states in the last step 

        Args:
            num_laserscans (int): [description]
            convert_on_last_robot_frame (bool): [description]

        Returns:
            np.ndarray: [description]
        """
        laserscans = []
        assert len(self._laser_scans)>0
        assert len(self._robot_states)>0
        max_data_frame = min(len(self._laser_scans), len(self._robot_states))
        assert max_data_frame > 0
        robot_pose_theta_latest = None
        # we assume the laser scan and the robot's state get published at the same rate
        for i, (laser_scan, robot_state) in enumerate(zip(reversed(self._laser_scans), reversed(self._robot_states))):
            if i >= max_data_frame or i >= num_laserscans:
                break
            theta = robot_state[0].theta
            if i == 0:
                robot_pose_theta_latest = robot_state[0].theta
            if convert_all_on_latest_robot_frame:
                offset = int((theta-robot_pose_theta_latest) /
                             self._lidar_angle_increment)
                laser_scan = np.roll(laser_scan, offset)
            laserscans.append(laser_scan)

        while len(laserscans) < num_laserscans:
            laserscans.append(laserscans[-1])

        return np.array(laserscans)

    def get_laserscans_in_map_frame(self,num_laserscans:int)->np.ndarray:
        laserscans = []
        assert len(self._laser_scans)>0
        assert len(self._robot_states)>0
        max_data_frame = min(len(self._laser_scans), len(self._robot_states))
        assert max_data_frame > 0
        robot_pose_theta_latest = None
        # we assume the laser scan and the robot's state get published at the same rate
        for i, (laser_scan, robot_state) in enumerate(zip(reversed(self._laser_scans), reversed(self._robot_states))):
            if i >= max_data_frame or i >= num_laserscans:
                break
            theta = robot_state[0].theta
            if i == 0:
                offset = int(theta /self._lidar_angle_increment)
                laser_scan = np.roll(laser_scan, offset)
            laserscans.append(laser_scan)

        while len(laserscans) < num_laserscans:
            laserscans.append(laserscans[-1])

        return np.array(laserscans)

    def get_subgoal_in_map_frame(self)->np.ndarray:
        assert self._subgoal is not None
        assert len(self._robot_states) > 0
        y_relative = self._subgoal.y - self._robot_states[-1][0].y
        x_relative = self._subgoal.x - self._robot_states[-1][0].x        
        rho = (x_relative**2+y_relative**2)**0.5
        theta = np.arctan2(y_relative, x_relative)
        return rho, theta

    def get_subgoal_in_latest_robot_frame(self) -> Tuple[float, float]:
        assert self._subgoal is not None
        assert len(self._robot_states) > 0
        y_relative = self._subgoal.y - self._robot_states[-1][0].y
        x_relative = self._subgoal.x - self._robot_states[-1][0].x
        rho = (x_relative**2+y_relative**2)**0.5
        theta = (np.arctan2(y_relative, x_relative) -
                 self._robot_states[-1][0].theta+4*np.pi) % (2*np.pi)-np.pi
        return rho, theta

    def get_globalgoal_in_map_frame(self, globalgoal:Optional[Pose2D]= None) -> Tuple[float, float]:
        assert len(self._robot_states) > 0
        if globalgoal is not None:
            self._globalgoal = globalgoal
        assert self._globalgoal is not None
        y_relative = self._globalgoal.y - self._robot_states[-1][0].y
        x_relative = self._globalgoal.x - self._robot_states[-1][0].x
        rho = (x_relative**2+y_relative**2)**0.5
        theta = np.arctan2(y_relative, x_relative)
        return rho, theta


    def get_globalgoal_in_latest_robot_frame(self, globalgoal:Optional[Pose2D]= None) -> Tuple[float, float]:
        assert len(self._robot_states) > 0
        if globalgoal is not None:
            self._globalgoal = globalgoal
        assert self._globalgoal is not None
        y_relative = self._globalgoal.y - self._robot_states[-1][0].y
        x_relative = self._globalgoal.x - self._robot_states[-1][0].x
        rho = (x_relative**2+y_relative**2)**0.5
        theta = (np.arctan2(y_relative, x_relative) -
                 self._robot_states[-1][0].theta+4*np.pi) % (2*np.pi)-np.pi
        return rho, theta

    def get_subgoal_pos(self) -> Pose2D:
        return self._subgoal

    def get_globalgoal_pos(self) -> Pose2D:
        return self._globalgoal

    def reset(self) -> bool:
        """reset the simulator and intermediate planner to get new subgoal by send some step world request         

        Returns:
            bool: if failed, return False
        """
        # DEBUG
        rospy.loginfo("Waypoint generator start reset run step world ")
        # normally making the simulator running for 1/self._robot_action_rate will make sure
        # the new states will be updated.
        # but it seems like the intermediate planner takes quite a lot steps to make a subgoal.
        delta = 0.1
        request = StepWorldRequest(2/self._robot_action_rate)
        try_times = 80

        while len(self._laser_scans) == 0 or len(self._robot_states)==0 or self._subgoal is None or self._old_subgoal is not None and \
                abs(self._subgoal.x-self._old_subgoal.x) < delta and \
                abs(self._subgoal.y-self._old_subgoal.y) < delta:
            self._step_world_srv(request)
            try_times -= 1
            if try_times == 0:
                if len(self._laser_scans) == 0:
                    print("no laser scan")
                if self._subgoal is None:
                    print("no subgoal")
                # DEBUG
                print(
                    "Waypoint generator reset failed, request to generate a new pair of start pos and goal pos")
                return False
        rospy.loginfo("Waypoint generator done step world")
        return True

    def clear_on_episode_start(self):
        """call it only when the episode is finished.
        """

        self.clear_on_step_start()
        self._globalgoal: Optional[Pose2D] = None
        # This parameter is needed because if we start a new
        # episode and have set the new start pos of the robot
        # and goal. The FSM still publish the old subgoal several
        # times. so we need this to comfirm the new subgoal is coming

        self._old_subgoal: Optional[Pose2D] = None
        self._subgoal: Optional[Pose2D] = None

        self._gloablplan = None
        self._robot_states = []
        self._laser_scans = []
        rospy.loginfo("episode start subgoal None")

    def clear_on_step_start(self):
        """call it only when the step is done.
        """

        self.important_event: Union[ObservationCollectorWP.Event.COLLISIONDETECTED,
                                    ObservationCollectorWP.Event.WAYPOINTARRIVAL, None] = None
        # save the last one
        if len(self._laser_scans) > 0:
            self._laser_scans = [self._laser_scans[-1]]
        if len(self._robot_states) > 0:
            self._robot_states = [self._robot_states[-1]]
        self._waypoint: Optional[Pose2D] = None

    def clear_subgoal(self):
        self._subgoal = None

    def set_waypoint(self, x, y):
        """set current waypoint, observation collector will use this to check whether the robot is close to this point and trigger the stop of the simulation/
        """
        self._waypoint = Pose2D()
        self._waypoint.x = x
        self._waypoint.y = y
        rospy.loginfo("set waypoint.")

    def _suspend_step_world(self):
        """ flatland provided a step world service, which running the simulator in given second.
        The observation colletor keeps checking the events, if a event such as collision happend,
        the simulator maybe required to suspend instantly.
        """
        self._stop_step_world_pub.publish(Empty())

    def _check_event(self, event: 'ObservationCollectorWP.Event', *args, **kwargs):
        def on_new_laserscan(scan: np.array):
            if scan.min() < self._robot_obstacle_min_dist:
                self._check_event(
                    ObservationCollectorWP.Event.COLLISIONDETECTED)

        def on_new_robot_state(robot_state: Tuple[Pose2D, Twist]):
            if self._waypoint is not None and (robot_state[0].x-self._waypoint.x)**2 +\
                    (robot_state[0].y-self._waypoint.y)**2 < self.robot_waypoint_min_dist**2:
                self._check_event(ObservationCollectorWP.Event.WAYPOINTARRIVAL)
            # DEBUG
            if self._waypoint is not None:
                rospy.loginfo(
                    f"Distance to waypoint: {((robot_state[0].x-self._waypoint.x)**2 + (robot_state[0].y-self._waypoint.y)**2)**0.5}")

        def on_collision_detected():
            self.important_event = ObservationCollectorWP.Event.COLLISIONDETECTED
            if self.is_train_mode:
                self._suspend_step_world()
            else:
                #TODO maybe log this? for example collision_times?
                pass

        def on_waypoint_arrival():
            self.important_event = ObservationCollectorWP.Event.WAYPOINTARRIVAL
            if self.is_train_mode:
                self._suspend_step_world()
            else:
                self.cv_important_event_deployment.notify()
        

        if event == ObservationCollectorWP.Event.NEWLASER:
            on_new_laserscan(*args, **kwargs)
        elif event == ObservationCollectorWP.Event.NEWROBOTSTATE:
            on_new_robot_state(*args, **kwargs)
        elif event == ObservationCollectorWP.Event.COLLISIONDETECTED:
            on_collision_detected()
        elif event == ObservationCollectorWP.Event.WAYPOINTARRIVAL:
            on_waypoint_arrival()
        else:
            raise NotImplementedError()

    def is_goal_reached(self):
        return self.important_event == ObservationCollectorWP.Event.GOALARRIVAL

    def _callback_laserscan(self, laserscan_msg):
        scan = np.array(laserscan_msg.ranges)
        scan[np.isnan(scan)] = laserscan_msg.range_max
        self._check_event(self.Event.NEWLASER, scan)
        self._laser_scans.append(scan)

    def _callback_robot_state(self, robotstate_msg):
        pose3d = robotstate_msg.pose.pose
        twist = robotstate_msg.twist.twist
        robot_state = (self.pose3D_to_pose2D(pose3d), twist)
        self._check_event(self.Event.NEWROBOTSTATE, robot_state)
        self._robot_states.append(robot_state)

    def _callback_globalgoal(self, globalgoal_msg):
        self._globalgoal = self.pose3D_to_pose2D(globalgoal_msg.pose)

    def _callback_subgoal(self, subgoal_msg):
        self._subgoal = self.pose3D_to_pose2D(subgoal_msg.pose)

    def _callback_globalplan(self, globalplan_msg):
        global_plan_2d = list(map(
            lambda p: ObservationCollector.pose3D_to_pose2D(p.pose), globalplan_msg.poses))
        global_plan_np = np.array(
            list(map(lambda p2d: [p2d.x, p2d.y], global_plan_2d)))
        self._gloablplan = global_plan_np

    def get_observation_space(self):
        return self.observation_space

    @staticmethod
    def _get_goal_pose_in_robot_frame(goal_pos: Pose2D, robot_pos: Pose2D):
        y_relative = goal_pos.y - robot_pos.y
        x_relative = goal_pos.x - robot_pos.x
        rho = (x_relative**2+y_relative**2)**0.5
        theta = (np.arctan2(y_relative, x_relative) -
                 robot_pos.theta+4*np.pi) % (2*np.pi)-np.pi
        return rho, theta

    @staticmethod
    def process_global_plan_msg(globalplan):
        global_plan_2d = list(map(
            lambda p: ObservationCollector.pose3D_to_pose2D(p.pose), globalplan.poses))
        global_plan_np = np.array(
            list(map(lambda p2d: [p2d.x, p2d.y], global_plan_2d)))
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


if __name__ == '__main__':

    state_collector = ObservationCollectorWP("sim1/", 360, 10)
