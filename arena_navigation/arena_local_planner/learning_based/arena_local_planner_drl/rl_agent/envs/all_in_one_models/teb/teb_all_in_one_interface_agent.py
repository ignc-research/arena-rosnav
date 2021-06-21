import subprocess
import time

import all_in_one_teb_interface.srv
import numpy as np
import rospy
import rosservice

import geometry_msgs.msg
from rl_agent.envs.all_in_one_models.model_base_class import ModelBase


class TebAllinOneInterfaceAgent(ModelBase):

    def __init__(self, name: str, ns: str, config_path: str):
        observation_information = {'robot_twist': True,
                                   'goal_in_robot_frame_xy': True}
        super().__init__(observation_information, name)
        self._ns = ns

        # Generate teb node
        package = 'all_in_one_teb_interface'
        launch_file = 'start_all_in_one_teb_node.launch'
        arg1 = "ns:=" + ns
        arg2 = "node_name:=" + name
        arg3 = "config_path:=" + config_path

        # Use subprocess to execute .launch file # TODO is this the right way? Use same logging window?
        subprocess.Popen(["roslaunch", package, launch_file, arg1, arg2, arg3])

        self._getCmdService = None

    def get_next_action(self, observation_dict: dict) -> np.ndarray:
        goal_xy = observation_dict['goal_in_robot_frame_xy']
        goal = geometry_msgs.msg.Pose2D(x=goal_xy[0], y=goal_xy[1], theta=0)
        request_msg = all_in_one_teb_interface.srv.TebServiceRequest(robot_velocity=observation_dict['robot_twist'],
                                                                     goal_pose=goal)
        response_msg: all_in_one_teb_interface.srv.TebServiceResponse = self._getCmdService(request_msg)
        if response_msg.costmaps_resetted:
            print("Warning: Teb has not found a feasable plan!")

        return np.array([response_msg.vel.linear.x, response_msg.vel.angular.z])

    def wait_for_agent(self) -> bool:
        service_name = "/" + self._ns + "/" + self._name + "/" + "getCmdVel"

        # wait until service is available
        service_list = rosservice.get_service_list()
        max_tries = 10
        for i in range(max_tries):
            if service_name in service_list:
                break
            else:
                time.sleep(0.3)

        if service_name not in service_list:
            return False
        else:
            self._getCmdService = rospy.ServiceProxy(service_name, all_in_one_teb_interface.srv.TebService)
            return True
