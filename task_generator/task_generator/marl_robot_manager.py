# from math import ceil, sqrt
from typing import Union

import math
import os

from torch.nn.modules.module import T
import rospy
import rospkg
import tf
import yaml

from flatland_msgs.srv import (
    MoveModel,
    MoveModelRequest,
    SpawnModelRequest,
    SpawnModel,
    StepWorld,
)
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import OccupancyGrid

from .utils import generate_freespace_indices, get_random_pos_on_map


class RobotManager:
    """
    A manager class using flatland provided services to spawn, move and delete Robot. Currently only one robot
    is managed
    """

    def __init__(
        self,
        ns: str,
        map_: OccupancyGrid,
        robot_yaml_path: str,
        robot_type: str,
        robot_id: str = "myrobot",
        timeout=20,
    ) -> None:
        """[summary]

        Args:
            ns(namespace): if ns == '', we will use global namespace
            map_ (OccupancyGrid): the map info
            robot_yaml_path (str): the file name of the base robot yaml file.

        """
        """SET A METHOD TO EXTRACT IF MARL IS BEIN REQUESTED"""
        MARL = rospy.get_param("num_robots") > 1

        self.ns = ns
        self.ns_prefix = "" if ns == "" else "/" + ns + "/"

        self.robot_id = robot_id
        self.robot_type = robot_type

        self.is_training_mode = rospy.get_param("/train_mode")
        self.step_size = rospy.get_param("step_size")

        self._get_robot_config(robot_yaml_path)
        robot_yaml_path = (
            self._generate_robot_config_with_adjusted_topics()
            if MARL
            else robot_yaml_path
        )

        # setup proxy to handle  services provided by flatland
        rospy.wait_for_service(f"{self.ns_prefix}move_model", timeout=timeout)
        rospy.wait_for_service(f"{self.ns_prefix}spawn_model", timeout=timeout)

        self._srv_move_model = rospy.ServiceProxy(
            f"{self.ns_prefix}move_model", MoveModel
        )
        self._srv_spawn_model = rospy.ServiceProxy(
            f"{self.ns_prefix}spawn_model", SpawnModel
        )
        # it's only needed in training mode to send the clock signal.
        self._step_world = rospy.ServiceProxy(f"{self.ns_prefix}step_world", StepWorld)

        # publisher
        goal_topic = (
            f"{self.ns_prefix}{self.robot_id}/goal" if MARL else f"{self.ns_prefix}goal"
        )
        self._goal_pub = rospy.Publisher(
            f"{goal_topic}", PoseStamped, queue_size=1, latch=True
        )

        self.update_map(map_)
        self._spawn_robot(robot_yaml_path)

        # remove temporary config file
        if MARL:
            os.remove(robot_yaml_path)

    def _spawn_robot(self, robot_yaml_path: str) -> None:
        request = SpawnModelRequest()
        request.yaml_path = robot_yaml_path
        request.name = self.robot_id
        request.ns = self.ns
        self._srv_spawn_model(request)

    def _get_robot_config(self, robot_yaml_path: str) -> None:
        """Get robot info e.g robot name, radius, Laser related infomation

        Args:
            robot_yaml_path ([type]): [description]
        """
        robot_config = os.path.join(
            rospkg.RosPack().get_path("arena_local_planner_drl"),
            "configs",
            "action_spaces",
            "default_settings_" + self.robot_type + ".yaml",
        )
        with open(robot_config, "r", encoding="utf-8") as target:
            config = yaml.load(target, Loader=yaml.FullLoader)
        self.ROBOT_RADIUS = config["robot"]["radius"]

        with open(robot_yaml_path, "r") as f:
            self._robot_data = yaml.safe_load(f)

            # get laser_update_rate
            for plugin in self._robot_data["plugins"]:
                if plugin["type"] == "Laser":
                    self.LASER_UPDATE_RATE = plugin.setdefault("update_rate", 1)

    def _generate_robot_config_with_adjusted_topics(self) -> str:
        """Generates a robot-specific config file (yaml) where the publication \
            and subscription topics are adjusted with the robot's namespace.

        Returns:
            str: Path of the robot-specific adjusted config file.

        Note:
        - The namespaces consist of: [simulation ns] / [robot name] / *topic*\
            e.g.: sim_1/myrobot/scan
        - The yaml files are temporarily dumped into *../simulator_setup/tmp_robot_configs*
        """
        self._robot_data["bodies"][0]["name"] = (
            self.robot_id + "_" + self._robot_data["bodies"][0]["name"]
        )

        for plugin in self._robot_data["plugins"]:
            if plugin["type"] == "DiffDrive":
                plugin["body"] = self._robot_data["bodies"][0]["name"]
                plugin["odom_frame_id"] = self.robot_id + "_" + plugin["odom_frame_id"]
                plugin["odom_pub"] = self.robot_id + "/" + plugin["odom_pub"]
                plugin["twist_sub"] = (
                    self.robot_id + "/" + plugin.get("twist_sub", "cmd_vel")
                )

            elif plugin["type"] == "Laser":
                plugin["topic"] = self.robot_id + "/" + plugin["topic"]
                plugin["body"] = self._robot_data["bodies"][0]["name"]
                plugin["frame"] = self.robot_id + "_" + plugin["frame"]

        tmp_folder_path = os.path.join(
            rospkg.RosPack().get_path("simulator_setup"), "tmp_robot_configs"
        )
        os.makedirs(tmp_folder_path, exist_ok=True)
        tmp_config_name = self.ns + self.robot_id + ".robot_config.yaml"
        tmp_config_path = os.path.join(tmp_folder_path, tmp_config_name)

        with open(tmp_config_path, "w") as fd:
            yaml.dump(self._robot_data, fd)

        return tmp_config_path

    def update_map(self, new_map: OccupancyGrid):
        self.map = new_map
        # a tuple stores the indices of the non-occupied spaces. format ((y,....),(x,...)
        self._free_space_indices = generate_freespace_indices(self.map)

    def move_robot(self, pose: Pose2D):
        """Move the robot to a given position.

        Args:
            pose (Pose2D): Target postion.
        """
        # call service move_model

        srv_request = MoveModelRequest()
        srv_request.name = self.robot_id
        srv_request.pose = pose

        # call service
        self._srv_move_model(srv_request)
        if self.is_training_mode:
            # a necessaray procedure to let the flatland publish the
            # laser,odom's Transformation, which are needed for creating
            # global path
            # assert self.step_size * \
            #     self.LASER_UPDATE_RATE == 1, f"TO run the traning successfully, make sure the laser_update_rate*step_size == 1 \
            #     \n\tcurrent step_size:\t {self.step_size}\n\tcurrent laser's update rate:\t {self.LASER_UPDATE_RATE} "
            for _ in range(math.ceil(1 / (self.step_size * self.LASER_UPDATE_RATE))):
                self._step_world()

    def set_start_pos_random(self):
        start_pos = Pose2D()
        start_pos.x, start_pos, start_pos.theta = get_random_pos_on_map(
            self._free_space_indices, self.map, self.ROBOT_RADIUS * 2
        )
        self.move_robot(start_pos)

    def set_start_pos_goal_pos(
        self,
        start_pos: Union[Pose2D, None] = None,
        goal_pos: Union[Pose2D, None] = None,
        min_dist=1,
        forbidden_zones: Union[list, None] = None,
    ):
        """Set up start position and the goal postion. Path validation checking will be conducted. If it failed, an
        exception will be raised.

        Args:
            start_pos (Union[Pose2D,None], optional): start position. if None, it will be set randomly. Defaults to None.
            goal_pos (Union[Pose2D,None], optional): [description]. if None, it will be set randomly .Defaults to None.
            min_dist (float): minimum distance between start_pos and goal_pos
            forbidden_zones (list): a list of tuples with the format (x,y,r), where the the robot should not be reset.
        Exception:
            Exception("can not generate a path with the given start position and the goal position of the robot")
        """

        def dist(x1, y1, x2, y2):
            return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

        if start_pos is None or goal_pos is None:
            # if any of them need to be random generated, we set a higher threshold,otherwise only try once
            max_try_times = 20
        else:
            max_try_times = 1

        i_try = 0
        start_pos_ = None
        goal_pos_ = None
        while i_try < max_try_times:

            if start_pos is None:
                start_pos_ = Pose2D()
                (start_pos_.x, start_pos_.y, start_pos_.theta,) = get_random_pos_on_map(
                    self._free_space_indices,
                    self.map,
                    self.ROBOT_RADIUS * 2,
                    forbidden_zones=forbidden_zones,
                )
            else:
                start_pos_ = start_pos

            if goal_pos is None:
                goal_pos_ = Pose2D()
                (goal_pos_.x, goal_pos_.y, goal_pos_.theta,) = get_random_pos_on_map(
                    self._free_space_indices, self.map, self.ROBOT_RADIUS * 2
                )
            else:
                goal_pos_ = goal_pos

            if dist(start_pos_.x, start_pos_.y, goal_pos_.x, goal_pos_.y) < min_dist:
                i_try += 1
                continue
            # move the robot to the start pos
            self.move_robot(start_pos_)
            try:
                # publish the goal, if the gobal plath planner can't generate a path, a, exception will be raised.
                self.publish_goal(goal_pos_.x, goal_pos_.y, goal_pos_.theta)
                break
            except rospy.ServiceException:
                i_try += 1
        if i_try == max_try_times:
            # TODO Define specific type of Exception
            raise rospy.ServiceException(
                "can not generate a path with the given start position and the goal position of the robot"
            )
        else:
            return start_pos_, goal_pos_

    def publish_goal(self, x, y, theta):
        """
        Publishing goal (x, y, theta)
        :param x x-position of the goal
        :param y y-position of the goal
        :param theta theta-position of the goal
        """
        goal = PoseStamped()
        goal.header.stamp = rospy.get_rostime()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        goal.pose.orientation.w = quaternion[0]
        goal.pose.orientation.x = quaternion[1]
        goal.pose.orientation.y = quaternion[2]
        goal.pose.orientation.z = quaternion[3]
        self._goal_pub.publish(goal)

    def __mean_square_dist_(self, x, y):
        return math.sqrt(math.pow(x, 2) + math.pow(y, 2))
