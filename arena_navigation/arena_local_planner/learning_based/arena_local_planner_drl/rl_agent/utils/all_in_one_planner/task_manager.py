import json
import os
import subprocess

import nav_msgs.srv
import rospkg
import rospy
from nav_msgs.srv import GetMap
from task_generator.obstacles_manager import ObstaclesManager
from task_generator.robot_manager import RobotManager
from task_generator.tasks import RandomTask


class TaskManager:

    def __init__(self, ns: str, reset_map_interval: int, paths: dict):
        self.ns = ns
        self.task = self._get_random_task(paths)
        self.resetMap_interval = reset_map_interval
        self.current_iteration = 0
        self._request_new_map = rospy.ServiceProxy("/" + self.ns + "/new_map", GetMap)

    def reset(self):
        self.current_iteration += 1
        if self.current_iteration % self.resetMap_interval == 0:
            self.current_iteration = 0
            self._update_map()
        self.task.reset()

    def _get_random_task(self, paths: dict):
        config_path = paths['map_parameters']
        with open(config_path, 'r') as params_json:
            map_params = json.load(params_json)

        assert map_params is not None, "Error: Map parameter file cannot be found!"

        numb_static_obst = map_params['numb_static_obstacles']
        numb_dyn_obst = map_params['numb_dynamic_obstacles']
        map_type = map_params['type']

        self._start_map_generator_node(map_type)

        service_client_get_map = rospy.ServiceProxy('/' + self.ns + '/static_map', GetMap)
        map_response = service_client_get_map()
        models_folder_path = rospkg.RosPack().get_path('simulator_setup')
        self.robot_manager = RobotManager(self.ns, map_response.map, os.path.join(
            models_folder_path, 'robot', "myrobot.model.yaml"))
        self.obstacles_manager = ObstaclesManager(self.ns, map_response.map)
        rospy.set_param("/task_mode", "random")
        numb_obst = numb_static_obst + numb_dyn_obst
        prob_dyn_obst = float(numb_dyn_obst) / numb_obst
        self.obstacles_manager.register_random_obstacles(numb_obst, prob_dyn_obst)
        return RandomTask(self.obstacles_manager, self.robot_manager)

    def _start_map_generator_node(self, map_type: str):
        package = 'simulator_setup'
        launch_file = 'map_generator.launch'
        arg1 = "ns:=" + self.ns
        arg2 = "type:=" + map_type

        # Use subprocess to execute .launch file
        self._global_planner_process = subprocess.Popen(["roslaunch", package, launch_file, arg1, arg2],
                                                        stdout=subprocess.DEVNULL)

    def _update_map(self):
        new_map = self._request_new_map(nav_msgs.srv.GetMapRequest())
        self.obstacles_manager.update_map(new_map.map)
        self.robot_manager.update_map(new_map.map)
