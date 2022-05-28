from typing import Type, Union, List

import json
import os
import rospkg
import rospy
import yaml

from abc import ABC, abstractmethod
from enum import Enum, unique
from threading import Lock
from filelock import FileLock

from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from std_msgs.msg import Bool

from .obstacles_manager import ObstaclesManager
from .robot_manager import RobotManager


class ABSMARLTask(ABC):
    """An abstract class for the DRL agent navigation tasks."""

    def __init__(
        self,
        obstacles_manager: ObstaclesManager,
        robot_manager: List[RobotManager],
    ):
        self.obstacles_manager = obstacles_manager
        self.robot_manager = robot_manager
        self._service_client_get_map = rospy.ServiceProxy("/static_map", GetMap)
        self._map_lock = Lock()
        rospy.Subscriber("/map", OccupancyGrid, self._update_map)
        # a mutex keep the map is not unchanged during reset task.

    @abstractmethod
    def reset(self):
        """
        Funciton to reset the task/scenery. Make sure that _map_lock is used.
        """

    def _update_map(self, map_: OccupancyGrid):
        with self._map_lock:
            self.obstacles_manager.update_map(map_)
            for manager in self.robot_manager:
                manager.update_map(map_)


class RandomMARLTask(ABSMARLTask):
    """Sets a randomly drawn start and goal position for each robot episodically."""

    def __init__(
        self,
        obstacles_manager: ObstaclesManager,
        robot_manager: List[RobotManager],
    ):
        super().__init__(obstacles_manager, robot_manager)
        self._num_robots = len(self.robot_manager)

    def reset(self):
        """[summary]"""
        with self._map_lock:
            max_fail_times = 3
            fail_times = 0
            while fail_times < max_fail_times:
                try:
                    starts, goals = [None] * self._num_robots, [
                        None
                    ] * self._num_robots
                    for i, manager in enumerate(self.robot_manager):
                        start_pos, goal_pos = manager.set_start_pos_goal_pos(
                            forbidden_zones=starts
                        )
                        starts[i] = (
                            start_pos.x,
                            start_pos.y,
                            manager.ROBOT_RADIUS * 1.5,
                        )
                        goals[i] = (
                            goal_pos.x,
                            goal_pos.y,
                            manager.ROBOT_RADIUS * 1.5,
                        )
                    self.obstacles_manager.reset_pos_obstacles_random(
                        forbidden_zones=starts + goals
                    )
                    break
                except rospy.ServiceException as e:
                    rospy.logwarn(repr(e))
                    fail_times += 1
            if fail_times == max_fail_times:
                raise Exception("reset error!")


class StagedMARLRandomTask(RandomMARLTask):
    """
    Enforces the paradigm of curriculum learning.
    The training stages are defined in 'training_curriculum.yaml'
    """

    def __init__(
        self,
        ns: str,
        obstacles_manager: ObstaclesManager,
        robot_manager: List[RobotManager],
        start_stage: int = 1,
        PATHS=None,
    ) -> None:
        super().__init__(obstacles_manager, robot_manager)
        self.ns = ns
        self.ns_prefix = "" if ns == "" else "/" + ns + "/"

        self._curr_stage = start_stage
        self._stages = {}
        self._PATHS = PATHS
        self._read_stages_from_yaml()

        # check start stage format
        if not isinstance(start_stage, int):
            raise ValueError("Given start_stage not an Integer!")
        if self._curr_stage < 1 or self._curr_stage > len(self._stages):
            raise IndexError(
                "Start stage given for training curriculum out of bounds! Has to be between {1 to %d}!"
                % len(self._stages)
            )
        rospy.set_param("/curr_stage", self._curr_stage)

        # hyperparamters.json location
        self.json_file = os.path.join(
            self._PATHS.get("model"), "hyperparameters.json"
        )
        assert os.path.isfile(self.json_file), (
            "Found no 'hyperparameters.json' at %s" % self.json_file
        )
        self._lock_json = FileLock(self.json_file + ".lock")

        # subs for triggers
        self._sub_next = rospy.Subscriber(
            f"{self.ns_prefix}next_stage", Bool, self.next_stage
        )
        self._sub_previous = rospy.Subscriber(
            f"{self.ns_prefix}previous_stage", Bool, self.previous_stage
        )

        self._initiate_stage()

    def next_stage(self, *args, **kwargs):
        if self._curr_stage < len(self._stages):
            self._curr_stage = self._curr_stage + 1
            self._initiate_stage()

            if self.ns == "eval_sim":
                rospy.set_param("/curr_stage", self._curr_stage)
                with self._lock_json:
                    self._update_curr_stage_json()

                if self._curr_stage == len(self._stages):
                    rospy.set_param("/last_stage_reached", True)
        else:
            print(
                f"({self.ns}) INFO: Tried to trigger next stage but already reached last one"
            )

    def previous_stage(self, *args, **kwargs):
        if self._curr_stage > 1:
            rospy.set_param("/last_stage_reached", False)

            self._curr_stage = self._curr_stage - 1
            self._initiate_stage()

            if self.ns == "eval_sim":
                rospy.set_param("/curr_stage", self._curr_stage)
                with self._lock_json:
                    self._update_curr_stage_json()
        else:
            print(
                f"({self.ns}) INFO: Tried to trigger previous stage but already reached first one"
            )

    def _initiate_stage(self):
        self._remove_obstacles()

        static_obstacles = self._stages[self._curr_stage]["static"]
        dynamic_obstacles = self._stages[self._curr_stage]["dynamic"]

        self.obstacles_manager.register_random_static_obstacles(
            self._stages[self._curr_stage]["static"]
        )
        self.obstacles_manager.register_random_dynamic_obstacles(
            self._stages[self._curr_stage]["dynamic"]
        )

        print(
            f"({self.ns}) Stage {self._curr_stage}:"
            f"Spawning {static_obstacles} static and {dynamic_obstacles} dynamic obstacles!"
        )

    def _read_stages_from_yaml(self):
        file_location = self._PATHS.get("curriculum")
        if os.path.isfile(file_location):
            with open(file_location, "r") as file:
                self._stages = yaml.load(file, Loader=yaml.FullLoader)
            assert isinstance(
                self._stages, dict
            ), "'training_curriculum.yaml' has wrong fromat! Has to encode dictionary!"
        else:
            raise FileNotFoundError(
                "Couldn't find 'training_curriculum.yaml' in %s "
                % self._PATHS.get("curriculum")
            )

    def _update_curr_stage_json(self):
        with open(self.json_file, "r") as file:
            hyperparams = json.load(file)
        try:
            hyperparams["curr_stage"] = self._curr_stage
        except Exception as e:
            raise Warning(
                f" {e} \n Parameter 'curr_stage' not found in 'hyperparameters.json'!"
            )
        else:
            with open(self.json_file, "w", encoding="utf-8") as target:
                json.dump(hyperparams, target, ensure_ascii=False, indent=4)

    def _remove_obstacles(self):
        self.obstacles_manager.remove_obstacles()


@unique
class ARENA_TASKS(Enum):
    MANUAL = "manual"
    RANDOM = "random"
    STAGED = "staged"
    SCENARIO = "scenario"


def get_mode(mode: str) -> ARENA_TASKS:
    return ARENA_TASKS(mode)


def get_MARL_task(
    ns: str,
    mode: str,
    robot_names: List[str],
    PATHS: dict,
    start_stage: int = 1,
) -> ABSMARLTask:
    """Function to return desired navigation task manager.

    Args:
        ns (str): Environments' ROS namespace. There should only be one env per ns.
        mode (str): avigation task mode for the agents. Modes to chose from: ['random', 'staged']. \
            Defaults to "random".
        robot_names (List[str]): List containing all robots' names in order to address the right namespaces.
        start_stage (int, optional): Starting difficulty level for the learning curriculum. Defaults to 1.
        PATHS (dict, optional): Dictionary containing program related paths. Defaults to None.

    Raises:
        NotImplementedError: The manual task mode is currently not implemented.
        NotImplementedError: The scenario task mode is currently not implemented.

    Returns:
        ABSMARLTask: A task manager instance.
    """
    assert type(robot_names) is list

    task_mode = get_mode(mode)

    # get the map
    service_client_get_map = rospy.ServiceProxy("/static_map", GetMap)
    map_response = service_client_get_map()

    # use rospkg to get the path where the model config yaml file stored
    models_folder_path = rospkg.RosPack().get_path("simulator_setup")

    # robot's yaml file is needed to get its configurations etc.
    base_robot_yaml = os.path.join(
        models_folder_path, "robot", "myrobot.model.yaml"
    )

    robot_manager = [
        RobotManager(
            ns=ns,
            map_=map_response.map,
            robot_yaml_path=base_robot_yaml,
            robot_name=name,
        )
        for name in robot_names
    ]

    obstacles_manager = ObstaclesManager(ns, map_response.map)

    task = None
    if task_mode == ARENA_TASKS.MANUAL:
        raise NotImplementedError
    if task_mode == ARENA_TASKS.RANDOM:
        rospy.set_param("/task_mode", "random")
        obstacles_manager.register_random_obstacles(10, 1.0)
        task = RandomMARLTask(obstacles_manager, robot_manager)
    if task_mode == ARENA_TASKS.STAGED:
        rospy.set_param("/task_mode", "staged")
        task = StagedMARLRandomTask(
            ns, obstacles_manager, robot_manager, start_stage, PATHS
        )
    if task_mode == ARENA_TASKS.SCENARIO:
        raise NotImplementedError
    return task
