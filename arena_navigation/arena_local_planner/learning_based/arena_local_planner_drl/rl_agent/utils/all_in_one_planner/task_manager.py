import json
import os
import time

import nav_msgs.srv
import rospkg
import rospy
import std_srvs.srv
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
        config_path = paths['all_in_one_parameters']
        with open(config_path, 'r') as params_json:
            config_data = json.load(params_json)

        assert config_data is not None, "Error: All in one parameter file cannot be found!"

        if 'map' in config_data:
            map_params = config_data['map']
            numb_static_obst = map_params['numb_static_obstacles']
            numb_dyn_obst = map_params['numb_dynamic_obstacles']
        else:
            rospy.logwarn("No map parameters found in config file. Use 18 dynamic and 0 static obstacles.")
            numb_static_obst = 0
            numb_dyn_obst = 18

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

    def _update_map(self):
        new_map = self._request_new_map(nav_msgs.srv.GetMapRequest())
        self.obstacles_manager.update_map(new_map.map)
        self.robot_manager.update_map(new_map.map)
