import math
import random
from typing import Union
import re
import yaml
import numpy as np
import os
import warnings
from flatland_msgs.srv import DeleteModel, DeleteModelRequest
from flatland_msgs.srv import SpawnModel, SpawnModelRequest
from flatland_msgs.srv import MoveModel, MoveModelRequest
from flatland_msgs.srv import StepWorld
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D, Twist, Point
from pedsim_srvs.srv import SpawnPeds
from pedsim_msgs.msg import Ped
from std_srvs.srv import SetBool, Empty
import rospy
import rospkg
import shutil

class PedManager:
    def __respawn_peds(self, peds):
        """
        Spawning one pedestrian in the simulation. The type of pedestrian is randomly decided here.
        TODO: the task generator later can decide the number of the agents
        ADULT = 0, CHILD = 1, ROBOT = 2, ELDER = 3,
        ADULT_AVOID_ROBOT = 10, ADULT_AVOID_ROBOT_REACTION_TIME = 11
        :param  start_pos start position of the pedestrian.
        :param  wps waypoints the pedestrian is supposed to walk to.
        :param  id id of the pedestrian.
        """
        srv = SpawnPeds()
        srv.peds = []
        # print(peds)
        self.agent_topic_str=''
        for i, ped in enumerate(peds):
            elements = [0, 1, 3]
            # probabilities = [0.4, 0.3, 0.3] np.random.choice(elements, 1, p=probabilities)[0]
            self.__ped_type=elements[i%3]
            if  self.__ped_type==0:
                self.agent_topic_str+=f',{self.ns_prefix}pedsim_agent_{i+1}/dynamic_human'
                self.__ped_file=os.path.join(rospkg.RosPack().get_path(
                'simulator_setup'), 'dynamic_obstacles/person_two_legged.model.yaml')
            elif self.__ped_type==1:
                self.agent_topic_str+=f',{self.ns_prefix}pedsim_agent_{i+1}/dynamic_child'
                self.__ped_file=os.path.join(rospkg.RosPack().get_path(
                'simulator_setup'), 'dynamic_obstacles/person_two_legged_child.model.yaml')
            else:
                # print('elder')
                self.agent_topic_str+=f',{self.ns_prefix}pedsim_agent_{i+1}/dynamic_elder'
                self.__ped_file=os.path.join(rospkg.RosPack().get_path(
                'simulator_setup'), 'dynamic_obstacles/person_single_circle_elder.model.yaml')
            msg = Ped()
            msg.id = ped[0]
            msg.pos = Point()
            msg.pos.x = ped[1][0]
            msg.pos.y = ped[1][1]
            msg.pos.z = ped[1][2]
            msg.type = self.__ped_type
            msg.number_of_peds = 1
            msg.yaml_file = self.__ped_file
            msg.waypoints = []
            for pos in ped[2]:
                p = Point()
                p.x = pos[0]
                p.y = pos[1]
                p.z = pos[2]
                msg.waypoints.append(p)
            srv.peds.append(msg)

        max_num_try = 2
        i_curr_try = 0
        while i_curr_try < max_num_try:
            # try to call service
            response=self.__respawn_peds_srv.call(srv.peds)
            # response=self.__spawn_ped_srv.call(srv.peds)
            if not response.finished:  # if service not succeeds, do something and redo service
                rospy.logwarn(
                    f"spawn human failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
                # rospy.logwarn(response.message)
                i_curr_try += 1
            else:
                break            
        # print("reached here respawn_humans")
        self.__peds = peds        
        rospy.set_param(f'{self.ns_prefix}agent_topic_string', self.agent_topic_str)
        return


if __name__ == '__main__':
    ped_manager = PedManager()
