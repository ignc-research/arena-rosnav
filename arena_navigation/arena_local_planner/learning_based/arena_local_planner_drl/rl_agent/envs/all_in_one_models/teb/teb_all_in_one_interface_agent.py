import subprocess
import time

import all_in_one_teb_interface.srv
import numpy as np
import rospy
import rosservice
from rl_agent.envs.all_in_one_models.model_base_class import ModelBase


class TebAllinOneInterfaceAgent(ModelBase):

    def __init__(self, name: str, ns: str, config_path: str):
        observation_information = {'robot_twist': True,
                                   'goal_map_frame': True}
        super().__init__(observation_information, name)
        self._ns = ns

        # Generate teb node
        package = 'all_in_one_teb_interface'
        launch_file = 'start_all_in_one_teb_node.launch'
        arg1 = "ns:=" + ns
        arg2 = "node_name:=" + name
        arg3 = "config_path:=" + config_path

        # Use subprocess to execute .launch file # TODO is this the right way? Use same logging window?
        self._teb_process = subprocess.Popen(["roslaunch", package, launch_file, arg1, arg2, arg3])

        self._getCmdService = None
        self._resetCostmapService = None

        self._teb_rdy = False

    def get_next_action(self, observation_dict: dict) -> np.ndarray:
        goal = observation_dict['goal_map_frame']
        request_msg = all_in_one_teb_interface.srv.TebServiceRequest(robot_velocity=observation_dict['robot_twist'],
                                                                     goal_pose=goal)
        response_msg: all_in_one_teb_interface.srv.TebServiceResponse = self._getCmdService(request_msg)

        # TODO make extended information available via parameter?
        # if not response_msg.successful:
        #     rospy.logwarn("Teb has not found a feasable plan!")
        # if response_msg.successful and response_msg.costmaps_resetted:
        #     rospy.logwarn("Teb linear velocity low, costmap was reseted.")

        return np.array([response_msg.vel.linear.x, response_msg.vel.angular.z])

    def wait_for_agent(self) -> bool:
        service_name_cmdvel = "/" + self._ns + "/" + self._name + "/" + "getCmdVel"
        service_name_resetcostmap = "/" + self._ns + "/" + self._name + "/" + "resetCostmap"

        # wait until service is available
        service_list = rosservice.get_service_list()
        max_tries = 10
        for i in range(max_tries):
            if service_name_cmdvel in service_list and service_name_resetcostmap in service_list:
                break
            else:
                time.sleep(0.3)

        if service_name_cmdvel not in service_list:
            self._teb_rdy = False
            return False
        else:
            self._getCmdService = rospy.ServiceProxy(service_name_cmdvel, all_in_one_teb_interface.srv.TebService,
                                                     persistent=True)
            self._resetCostmapService = rospy.ServiceProxy(service_name_resetcostmap,
                                                           all_in_one_teb_interface.srv.ResetCostmap, persistent=False)
            self._teb_rdy = True
            return True

    def reset(self):
        """
        Send service request to clear costmap.
        """
        if self._teb_rdy:
            self._resetCostmapService()

    def close(self):
        self._teb_process.terminate()
