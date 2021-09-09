import os
from abc import ABC, abstractmethod
from threading import Condition, Lock
from filelock import FileLock

import rospy
import rospkg
import json
import yaml
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Pose2D
from rospy.exceptions import ROSException

from std_msgs.msg import Bool

from .obstacles_manager import ObstaclesManager
from .robot_manager import RobotManager
from pathlib import Path

from typing import List
from std_srvs.srv import Trigger
from pedsim_srvs.srv import SpawnPeds
from pedsim_srvs.srv import SpawnInteractiveObstacles
from pedsim_srvs.srv import SpawnObstacle
from pedsim_msgs.msg import Ped
from pedsim_msgs.msg import InteractiveObstacle
from pedsim_msgs.msg import LineObstacles
from pedsim_msgs.msg import LineObstacle

import sys
arena_tools_path = Path(__file__).parent / ".." / ".." / ".." / "forks" / "arena-tools"
sys.path.append(str(arena_tools_path))
from ArenaScenario import *


class StopReset(Exception):
    """Raised when The Task can not be reset anymore """


class ABSTask(ABC):
    """An abstract class, all tasks must implement reset function.

    """

    def __init__(self, obstacles_manager: ObstaclesManager, robot_manager: RobotManager):
        self.obstacles_manager = obstacles_manager
        self.robot_manager = robot_manager
        self._service_client_get_map = rospy.ServiceProxy("/" + obstacles_manager.ns + '/static_map', GetMap)
        self._map_lock = Lock()
        rospy.Subscriber("/" + obstacles_manager.ns + '/map', OccupancyGrid, self._update_map)
        # a mutex keep the map is not unchanged during reset task.

    @abstractmethod
    def reset(self):
        """
        a funciton to reset the task. Make sure that _map_lock is used.
        """

    def _update_map(self, map_: OccupancyGrid):
        with self._map_lock:
            self.obstacles_manager.update_map(map_)
            self.robot_manager.update_map(map_)


class RandomTask(ABSTask):
    """ Evertime the start position and end position of the robot is reset.
    """

    def __init__(self, obstacles_manager: ObstaclesManager, robot_manager: RobotManager):
        super().__init__(obstacles_manager, robot_manager)

    def reset(self):
        """[summary]
        """
        with self._map_lock:
            max_fail_times = 10
            fail_times = 0
            while fail_times < max_fail_times:
                try:
                    start_pos, goal_pos = self.robot_manager.set_start_pos_goal_pos()
                    self.obstacles_manager.reset_pos_obstacles_random(
                        forbidden_zones=[
                            (start_pos.x,
                                start_pos.y,
                                self.robot_manager.ROBOT_RADIUS * 4),
                            (goal_pos.x,
                                goal_pos.y,
                                self.robot_manager.ROBOT_RADIUS * 4)])
                    break
                except rospy.ServiceException as e:
                    rospy.logwarn(repr(e))
                    fail_times += 1
            if fail_times == max_fail_times:
                raise Exception("reset error!")


class ManualTask(ABSTask):
    """randomly spawn obstacles and user can mannually set the goal postion of the robot
    """

    def __init__(self,obstacles_manager: ObstaclesManager, robot_manager: RobotManager):
        super().__init__(obstacles_manager, robot_manager)
        # subscribe
        rospy.Subscriber(f'{self.ns}manual_goal', Pose2D, self._set_goal_callback)
        self._goal = Pose2D()
        self._new_goal_received = False
        self._manual_goal_con = Condition()

    def reset(self):
        while True:
            with self._map_lock:
                self.obstacles_manager.reset_pos_obstacles_random()
                self.robot_manager.set_start_pos_random()
                with self._manual_goal_con:
                    # the user has 60s to set the goal, otherwise all objects will be reset.
                    self._manual_goal_con.wait_for(
                        self._new_goal_received, timeout=60)
                    if not self._new_goal_received:
                        raise Exception(
                            "TimeOut, User does't provide goal position!")
                    else:
                        self._new_goal_received = False
                    try:
                        # in this step, the validation of the path will be checked
                        self.robot_manager.publish_goal(
                            self._goal.x, self._goal.y, self._goal.theta)
                    except Exception as e:
                        rospy.logwarn(repr(e))

    def _set_goal_callback(self, goal: Pose2D):
        with self._manual_goal_con:
            self._goal = goal
            self._new_goal_received = True
        self._manual_goal_con.notify()


class StagedRandomTask(RandomTask):
    def __init__(self, ns: str, obstacles_manager: ObstaclesManager, robot_manager: RobotManager, start_stage: int = 1, PATHS=None):
        super().__init__(obstacles_manager, robot_manager)
        self.ns = ns
        self.ns_prefix = "" if ns == '' else "/"+ns+"/"

        self._curr_stage = start_stage
        self._stages = dict()
        self._PATHS = PATHS
        self._read_stages_from_yaml()

        # check start stage format
        if not isinstance(start_stage, int):
            raise ValueError(
                "Given start_stage not an Integer!")
        if (self._curr_stage < 1 or 
            self._curr_stage > len(self._stages)):
            raise IndexError(
                "Start stage given for training curriculum out of bounds! Has to be between {1 to %d}!" % len(self._stages))
        rospy.set_param("/curr_stage", self._curr_stage)

        # hyperparamters.json location
        self.json_file = os.path.join(
            self._PATHS.get('model'), "hyperparameters.json")
        assert os.path.isfile(self.json_file), "Found no 'hyperparameters.json' at %s"
        self._lock_json = FileLock(self.json_file + ".lock")

        # subs for triggers
        self._sub_next = rospy.Subscriber(f"{self.ns_prefix}next_stage", Bool, self.next_stage)
        self._sub_previous = rospy.Subscriber(f"{self.ns_prefix}previous_stage", Bool, self.previous_stage)

        self._initiate_stage()

    def next_stage(self, msg: Bool):
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
                f"({self.ns}) INFO: Tried to trigger next stage but already reached last one")

    def previous_stage(self, msg: Bool):
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
                f"({self.ns}) INFO: Tried to trigger previous stage but already reached first one")

    def _initiate_stage(self):
        self._remove_obstacles()
        
        static_obstacles = self._stages[self._curr_stage]['static']
        dynamic_obstacles = self._stages[self._curr_stage]['dynamic']

        self.obstacles_manager.register_random_static_obstacles(
            self._stages[self._curr_stage]['static'])
        self.obstacles_manager.register_random_dynamic_obstacles(
            self._stages[self._curr_stage]['dynamic'])

        print(
            f"({self.ns}) Stage {self._curr_stage}: Spawning {static_obstacles} static and {dynamic_obstacles} dynamic obstacles!")

    def _read_stages_from_yaml(self):
        file_location = self._PATHS.get('curriculum')
        if os.path.isfile(file_location):
            with open(file_location, "r") as file:
                self._stages = yaml.load(file, Loader=yaml.FullLoader)
            assert isinstance(
                self._stages, dict), "'training_curriculum.yaml' has wrong fromat! Has to encode dictionary!"
        else:
            raise FileNotFoundError(
                "Couldn't find 'training_curriculum.yaml' in %s " % self._PATHS.get('curriculum'))

    def _update_curr_stage_json(self):
        with open(self.json_file, "r") as file:
            hyperparams = json.load(file)
        try:
            hyperparams['curr_stage'] = self._curr_stage
        except Exception:
            raise Warning(
                "Parameter 'curr_stage' not found in 'hyperparameters.json'!")
        else:
            with open(self.json_file, "w", encoding='utf-8') as target:
                json.dump(hyperparams, target,
                        ensure_ascii=False, indent=4)

    def _remove_obstacles(self):
        self.obstacles_manager.remove_obstacles()


class PedsimManager():
    def __init__(self):
        # spawn peds
        spawn_peds_service_name = "pedsim_simulator/spawn_peds"
        rospy.wait_for_service(spawn_peds_service_name, 6.0)
        self.spawn_peds_client = rospy.ServiceProxy(spawn_peds_service_name, SpawnPeds)
        # respawn peds
        respawn_peds_service_name = "pedsim_simulator/respawn_peds"
        rospy.wait_for_service(respawn_peds_service_name, 6.0)
        self.respawn_peds_client = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)
        # spawn interactive obstacles
        pawn_interactive_obstacles_service_name = "pedsim_simulator/spawn_interactive_obstacles"
        rospy.wait_for_service(pawn_interactive_obstacles_service_name, 6.0)
        self.spawn_interactive_obstacles_client = rospy.ServiceProxy(pawn_interactive_obstacles_service_name, SpawnInteractiveObstacles)
        # respawn interactive obstacles
        respawn_interactive_obstacles_service_name = "pedsim_simulator/respawn_interactive_obstacles"
        rospy.wait_for_service(respawn_interactive_obstacles_service_name, 6.0)
        self.respawn_interactive_obstacles_client = rospy.ServiceProxy(respawn_interactive_obstacles_service_name, SpawnInteractiveObstacles)
        # respawn interactive obstacles
        reset_all_peds_service_name = "pedsim_simulator/reset_all_peds"
        rospy.wait_for_service(reset_all_peds_service_name, 6.0)
        self.reset_all_peds_client = rospy.ServiceProxy(reset_all_peds_service_name, Trigger)

    def spawnPeds(self, peds: List[Ped]):
        res = self.spawn_peds_client.call(peds)
        print(res)

    def respawnPeds(self, peds: List[Ped]):
        res = self.respawn_peds_client.call(peds)
        print(res)

    def spawnInteractiveObstacles(self, obstacles: List[InteractiveObstacle]):
        res = self.spawn_interactive_obstacles_client.call(obstacles)
        print(res)

    def respawnInteractiveObstacles(self, obstacles: List[InteractiveObstacle]):
        res = self.respawn_interactive_obstacles_client.call(obstacles)
        print(res)

    def resetAllPeds(self):
        res = self.reset_all_peds_client.call()
        print(res)


class ScenarioTask(ABSTask):
    def __init__(self, obstacles_manager: ObstaclesManager, robot_manager: RobotManager, scenario_path: str):
        super().__init__(obstacles_manager, robot_manager)

        # load scenario from file
        self.scenario = ArenaScenario()
        self.scenario.loadFromFile(scenario_path)

        # setup pedsim agents
        self.pedsim_manager = None
        if len(self.scenario.pedsimAgents) > 0:
            self.pedsim_manager = PedsimManager()
            peds = [agent.getPedMsg() for agent in self.scenario.pedsimAgents]
            self.pedsim_manager.spawnPeds(peds)

        # setup static flatland obstacles
        for obstacle in self.scenario.staticObstacles:
            self.obstacles_manager._srv_spawn_model.call(
                obstacle.flatlandModel.path,
                obstacle.name,
                "static_obstacles",
                Pose2D(obstacle.pos[0], obstacle.pos[1], obstacle.angle)
            )

        self.reset_count = 0

    def reset(self):
        self.reset_count += 1
        info = {}
        with self._map_lock:
            # reset pedsim agents
            if self.pedsim_manager != None:
                self.pedsim_manager.resetAllPeds()

            # reset robot
            self.robot_manager.set_start_pos_goal_pos(
                Pose2D(self.scenario.robotPosition[0], self.scenario.robotPosition[1], 0),
                Pose2D(self.scenario.robotGoal[0], self.scenario.robotGoal[1], 0)
            )

            # fill info dict
            if self.reset_count == 1:
                info["new_scenerio_loaded"] = True
            else:
                info["new_scenerio_loaded"] = False
            info["robot_goal_pos"] = self.scenario.robotGoal
            info['num_repeats_curr_scene'] = self.reset_count
            info['max_repeats_curr_scene'] = 1000  # todo: implement max number of repeats for scenario
        return info


def get_scenario_file_format(path: str):
    path_ = Path(path)
    assert path_.is_file()
    data = json.load(path_.open())
    if "format" in data:
        if data["format"] == "arena-tools":
            return "arena-tools"
    else:
        return "scenerio"


def get_predefined_task(ns: str, mode="random", start_stage: int = 1, PATHS: dict = None):

    # TODO extend get_predefined_task(mode="string") such that user can choose between task, if mode is

    # check is it on traininig mode or test mode. if it's on training mode
    # flatland will provide an service called 'step_world' to change the simulation time
    # otherwise it will be bounded to real time.

    # either e.g. ns = 'sim1/' or ns = ''

    # get the map
    
    service_client_get_map = rospy.ServiceProxy('/static_map', GetMap)
    map_response = service_client_get_map()

    # use rospkg to get the path where the model config yaml file stored
    models_folder_path = rospkg.RosPack().get_path('simulator_setup')
    
    # robot's yaml file is needed to get its radius.
    robot_manager = RobotManager(ns, map_response.map, os.path.join(
        models_folder_path, 'robot', "myrobot.model.yaml"))
    
    obstacles_manager = ObstaclesManager(ns, map_response.map)
    
    # only generate 3 static obstaticles
    # obstacles_manager.register_obstacles(3, os.path.join(
    # models_folder_path, "obstacles", 'random.model.yaml'), 'static')
    # generate 5 static or dynamic obstaticles
    # obstacles_manager.register_random_obstacles(20, 0.4)

    # TODO In the future more Task will be supported and the code unrelated to
    # Tasks will be moved to other classes or functions.
    task = None
    if mode == "random":
        rospy.set_param("/task_mode", "random")
        obstacles_manager.register_random_obstacles(20, 0.4)
        task = RandomTask(obstacles_manager, robot_manager)
        print("random tasks requested")
    if mode == "manual":
        rospy.set_param("/task_mode", "manual")
        obstacles_manager.register_random_obstacles(20, 0.4)
        task = ManualTask(obstacles_manager, robot_manager)
        print("manual tasks requested")
    if mode == "staged":
        rospy.set_param("/task_mode", "staged")
        task = StagedRandomTask(
            ns, obstacles_manager, robot_manager, start_stage, PATHS)
    if mode == "scenario":
        rospy.set_param("/task_mode", "scenario")
        scenario_format = get_scenario_file_format(PATHS['scenario'])
        if scenario_format == "arena-tools":
            task = ScenarioTask(obstacles_manager, robot_manager, PATHS['scenario'])
        else:
            task = ScenerioTask(obstacles_manager, robot_manager, PATHS['scenario'])
    return task
