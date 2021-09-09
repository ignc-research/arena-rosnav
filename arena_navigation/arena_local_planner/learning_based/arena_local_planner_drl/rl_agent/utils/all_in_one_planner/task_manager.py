import json
import os
import random
import subprocess
import time
import rospkg
import rospy
import rosservice
from nav_msgs.srv import GetMap
from rospy import ServiceException
from task_generator.obstacles_manager import ObstaclesManager
from task_generator.robot_manager import RobotManager
from task_generator.tasks import *


from simulator_setup.srv import *


class TaskManager:

    def __init__(self, ns: str, reset_map_interval: int, paths: dict, run_scenario: bool):
        self.ns = ns
        self.run_scenario = run_scenario
        if self.run_scenario:
            self.task = get_predefined_task(ns, mode='scenario', start_stage=1, PATHS=paths)
        else:
            self.task = self._get_random_task(paths)
            self.resetMap_interval = reset_map_interval
            self.current_iteration = 0
            self._request_new_map = rospy.ServiceProxy("/" + self.ns + "/new_map", GetMapWithSeed)

    def reset(self, seed):
        if not self.run_scenario:
            self.current_iteration += 1
            if self.current_iteration % self.resetMap_interval == 0:
                self.current_iteration = 0
                self._update_map(seed)
            random.seed(seed)
        self.task.reset()

    def _get_random_task(self, paths: dict):
        config_path = paths['map_parameters']
        with open(config_path, 'r') as params_json:
            map_params = json.load(params_json)

        assert map_params is not None, "Error: Map parameter file cannot be found!"

        numb_static_obst = map_params['numb_static_obstacles']
        numb_dyn_obst = map_params['numb_dynamic_obstacles']
        map_type = map_params['type']
        if map_type == 'mixed':
            indoor_prob = map_params['indoor_prob']
        else:
            indoor_prob = 0

        self._start_map_generator_node(map_type, indoor_prob)

        service_client_get_map = rospy.ServiceProxy('/' + self.ns + '/static_map', GetMap)

        service_name = '/' + self.ns + '/static_map'
        service_list = rosservice.get_service_list()
        max_tries = 10
        for i in range(max_tries):
            if service_name in service_list:
                break
            else:
                time.sleep(1)

        map_response = service_client_get_map()
        models_folder_path = rospkg.RosPack().get_path('simulator_setup')
        self.robot_manager = RobotManager(self.ns, map_response.map, os.path.join(
            models_folder_path, 'robot', "myrobot.model.yaml"))
        self.obstacles_manager = ObstaclesManager(self.ns, map_response.map)
        rospy.set_param("/task_mode", "random")
        numb_obst = numb_static_obst + numb_dyn_obst
        if numb_obst != 0:
            prob_dyn_obst = float(numb_dyn_obst) / numb_obst
        else:
            prob_dyn_obst = 1
        self.obstacles_manager.register_random_obstacles(numb_obst, prob_dyn_obst)
        return RandomTask(self.obstacles_manager, self.robot_manager)

    def _start_map_generator_node(self, map_type: str, indoor_prob: float):
        package = 'simulator_setup'
        launch_file = 'map_generator.launch'
        arg1 = "ns:=" + self.ns
        arg2 = "type:=" + map_type
        arg3 = "indoor_prob:=" + str(indoor_prob)

        # Use subprocess to execute .launch file
        self._global_planner_process = subprocess.Popen(["roslaunch", package, launch_file, arg1, arg2, arg3],
                                                        stdout=subprocess.DEVNULL)

    def _update_map(self, seed: int):
        request = GetMapWithSeedRequest(seed=seed)
        try:
            new_map = self._request_new_map(request)
            self.obstacles_manager.update_map(new_map.map)
            self.robot_manager.update_map(new_map.map)
        except ServiceException:
            pass
