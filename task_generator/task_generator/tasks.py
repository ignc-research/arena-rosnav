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


class StopReset(Exception):
    """Raised when The Task can not be reset anymore """


class ABSTask(ABC):
    """An abstract class, all tasks must implement reset function.

    """

    def __init__(self, obstacles_manager: ObstaclesManager, robot_manager: RobotManager):
        self.obstacles_manager = obstacles_manager
        self.robot_manager = robot_manager
        self._service_client_get_map = rospy.ServiceProxy('/static_map', GetMap)
        self._map_lock = Lock()
        rospy.Subscriber('/map', OccupancyGrid, self._update_map)
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
            max_fail_times = 3
            fail_times = 0
            while fail_times < max_fail_times:
                try:
                    start_pos, goal_pos = self.robot_manager.set_start_pos_goal_pos()
                    self.obstacles_manager.reset_pos_obstacles_random(
                        forbidden_zones=[
                            (start_pos.x,
                                start_pos.y,
                                self.robot_manager.ROBOT_RADIUS),
                            (goal_pos.x,
                                goal_pos.y,
                                self.robot_manager.ROBOT_RADIUS)])
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
        assert os.path.isfile(self.json_file), "Found no 'hyperparameters.json' at %s" % json_file
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


class ScenerioTask(ABSTask):
    def __init__(self, obstacles_manager: ObstaclesManager, robot_manager: RobotManager, scenerios_json_path: str):
        """ The scenerio_json_path only has the "Scenerios" section, which contains a list of scenerios
        Args:
            scenerios_json_path (str): [description]
        """
        super().__init__(obstacles_manager, robot_manager)
        
        json_path = Path(scenerios_json_path)
        
        assert json_path.is_file() and json_path.suffix == ".json"
        json_data = json.load(json_path.open())
        
        self._scenerios_data = json_data["scenarios"]
        # current index of the scenerio
        self._idx_curr_scene = -1
        # The times of current scenerio repeated
        self._num_repeats_curr_scene = -1
        # The times of current scenerio need to be repeated
        self._max_repeats_curr_scene = 0

    def reset(self):
        info = {}
        with self._map_lock:
            if self._idx_curr_scene == -1 or self._num_repeats_curr_scene == self._max_repeats_curr_scene:
                self._set_new_scenerio()
                info["new_scenerio_loaded"] = True
            else:
                info["new_scenerio_loaded"] = False
                self.obstacles_manager.move_all_obstacles_to_start_pos_tween2()
            # reset robot
            robot_data = self._scenerios_data[self._idx_curr_scene]['robot']
            robot_start_pos = robot_data["start_pos"]
            robot_goal_pos = robot_data["goal_pos"]
            info["robot_goal_pos"] = robot_goal_pos
            self.robot_manager.set_start_pos_goal_pos(
                Pose2D(*robot_start_pos), Pose2D(*robot_goal_pos))
            self._num_repeats_curr_scene += 1
            info['num_repeats_curr_scene'] = self._num_repeats_curr_scene
            info['max_repeats_curr_scene'] = self._max_repeats_curr_scene
        return info

    def _set_new_scenerio(self):
        try:
            while True:
                self._idx_curr_scene += 1
                scenerio_data = self._scenerios_data[self._idx_curr_scene]
                scenerio_name = scenerio_data['scene_name']
                print(f"======================================================")
                print(f"Scenario '{scenerio_name}' loaded")
                print(f"======================================================")
                # use can set "repeats" to a non-positive value to disable the scenerio
                if scenerio_data["repeats"] > 0:
                    # set obstacles
                    self.obstacles_manager.remove_obstacles()
                    watchers_dict = scenerio_data.setdefault('watchers', [])
                    for obstacle_name, obstacle_data in scenerio_data["static_obstacles"].items():
                        if obstacle_data['shape'] == 'circle':
                            self.obstacles_manager.register_static_obstacle_circle(obstacle_data['x'],obstacle_data['y'],obstacle_data['radius'])
                        # vertices uses global coordinate system, the order of the vertices is doesn't matter
                        elif obstacle_data['shape'] == 'polygon': 
                            obstacle_vertices = np.array(
                                obstacle_data["vertices"], dtype=np.float)
                            self.obstacles_manager.register_static_obstacle_polygon(
                                obstacle_vertices)
                        else:
                            raise ValueError(f"Shape {obstacle_data['shape']} is not supported, supported shape 'circle' OR 'polygon'")

                    for obstacle_name, obstacle_data in scenerio_data["dynamic_obstacles"].items():
                        # currently dynamic obstacle only has circle shape
                        obstacle_radius = obstacle_data["obstacle_radius"]
                        linear_velocity = obstacle_data['linear_velocity']
                        # 3-elementary list
                        start_pos = obstacle_data["start_pos"]
                        waypoints = obstacle_data["waypoints"]
                        is_waypoint_relative = obstacle_data["is_waypoint_relative"]
                        mode = obstacle_data["mode"]
                        trigger_zones = []
                        if 'triggers' in obstacle_data:
                            for trigger in obstacle_data['triggers']:
                                if trigger not in watchers_dict:
                                    raise ValueError(
                                        f"For dynamic obstacle [{obstacle_name}] the trigger: {trigger} not found in the corresponding 'watchers' dict for scene {scenerio_name} ")
                                trigger_zones.append(
                                    watchers_dict[trigger]['pos']+[watchers_dict[trigger]['range']])
                        self.obstacles_manager.register_dynamic_obstacle_circle_tween2(
                            obstacle_name, obstacle_radius, linear_velocity, start_pos, waypoints, is_waypoint_relative, mode, trigger_zones)
                    # self.robot_
                    robot_data = scenerio_data["robot"]
                    robot_start_pos = robot_data["start_pos"]
                    robot_goal_pos = robot_data["goal_pos"]
                    self.robot_manager.set_start_pos_goal_pos(
                        Pose2D(*robot_start_pos), Pose2D(*robot_goal_pos))

                    self._num_repeats_curr_scene = 0
                    self._max_repeats_curr_scene = scenerio_data["repeats"]
                    break

        except IndexError as e:
            raise StopReset("All scenerios have been evaluated!") from e

    @staticmethod
    def generate_scenerios_json_example(dst_json_path: str):
        dst_json_path_ = Path(dst_json_path)
        dst_json_path_.parent.mkdir(parents=True, exist_ok=True)
        json_data = {}
        scene1 = {}
        scene2 = {}
        scene1['scene_name'] = 'scene_1'
        scene1['repeats'] = 2
        scene1_dynamic_obstacles = {}
        scene1_static_obstacles = {}
        scene1_robot = {'start_pos': [
            0.0, 0.0, 0.2], 'goal_pos': [4, 8, 0]}
        # trigger is optional, if it is not given, it will be trigged immediately
        scene1_dynamic_obstacles['dynamic_obs_0'] = {'obstacle_radius': 0.3, 'linear_velocity': 0.2, 'start_pos': [
            0, 3, 0], 'waypoints': [[0, 7, 0]], 'is_waypoint_relative': True, 'mode': 'yoyo', 'triggers': ['watcher_1']}
        scene1_dynamic_obstacles['dynamic_obs_1'] = {'obstacle_radius': 0.3, 'linear_velocity': 0.2, 'start_pos': [
            0, 4, 0], 'waypoints': [[8, 0, 0]], 'is_waypoint_relative': True, 'mode': 'yoyo', 'triggers': ['watcher_2']}
        # shape can be polygon or circle
        scene1_static_obstacles['static_obs_1'] = {
            'shape': 'polygon', 'vertices': [[2, 0.4], [2, 0.5], [5, 0.5], [5, 0.4]]}
        scene1_static_obstacles['static_obs_2'] = {
            'shape': 'polygon', 'vertices': [[3, 0.4], [3, 0.5], [7, 0.5]]}
        scene1_static_obstacles['static_obs_2'] = {
            'shape': 'circle', 'x': 4, 'y':5, 'radius':0.2} 
        scene1['dynamic_obstacles'] = scene1_dynamic_obstacles
        scene1['static_obstacles'] = scene1_static_obstacles
        scene1['robot'] = scene1_robot
        scene1['watchers'] = {'watcher_1': {
            'pos': [1, 1], 'range': 1}, 'watcher_2': {'pos': [5, 5], 'range': 2}}

        scene2['scene_name'] = 'scene_2'
        scene2['repeats'] = 1
        scene2_dynamic_obstacles = {}
        scene2_static_obstacles = {}
        scene2_robot = {'start_pos': [
            0.0, 0.1, 0.2], 'goal_pos': [1.5, 1.5, 0]}
        scene2_dynamic_obstacles['dynamic_obs_0'] = {'obstacle_radius': 0.3, 'linear_velocity': 0.2, 'start_pos': [
            7, 7, 0], 'waypoints': [[-4, 0, 0], [-4, -4, 0]], 'is_waypoint_relative': True, 'mode': 'yoyo'}
        scene2_dynamic_obstacles['dynamic_obs_1'] = {'obstacle_radius': 0.3, 'linear_velocity': 0.2, 'start_pos': [
            10, 3, 0], 'waypoints': [[0, 4, 0], [-5, 0, 0]], 'is_waypoint_relative': True, 'mode': 'yoyo'}
        scene2_static_obstacles['static_obs_1'] = {'shape':'polygon','vertices': [
            [1.2, 0.4], [1.2, 0.5], [0.75, 0.5], [0.75, 0.4]]}
        scene2['dynamic_obstacles'] = scene2_dynamic_obstacles
        scene2['static_obstacles'] = scene2_static_obstacles
        scene2['robot'] = scene2_robot
        scene2['watchers'] = {'watcher_1': {
            'pos': [1, 1], 'range': 4}, 'watcher_2': {'pos': [1, 1], 'range': 4}}
        json_data['scenerios'] = [scene1, scene2]
        json.dump(json_data, dst_json_path_.open('w'), indent=4)


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
        task = ScenerioTask(obstacles_manager, robot_manager,
            PATHS['scenario'])
    return task
