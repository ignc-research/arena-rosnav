#!/usr/bin/env python
import rospy
import os
import rospkg
import sys
import time
import copy
from std_srvs.srv import SetBool
from pedsim_srvs.srv import SpawnPeds
from pedsim_srvs.srv import SpawnInteractiveObstacles
from pedsim_msgs.msg import Ped
from pedsim_msgs.msg import InteractiveObstacle
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

def example1():
    rospack = rospkg.RosPack()
    respawn_peds_service_name = "pedsim_simulator/respawn_peds"
    rospy.wait_for_service(respawn_peds_service_name, 6.0)
    respawn_ped_srv = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)

    peds = []

    # ped1 = Ped()
    # ped1.id = 2
    # ped1.pos = Point(6, 3, 0)
    # ped1.type = 0
    # ped1.number_of_peds = 8
    # ped1.max_talking_distance = 3
    # ped1.chatting_probability = 0.1
    # ped1.vmax = 2.0
    # ped1.force_factor_desired = 5.0
    # ped1.force_factor_obstacle = 1.0
    # ped1.force_factor_social = 1.0
    # ped1.waypoint_mode = 1
    # ped1.waypoints = [Point(7, 2, 0.1), Point(7, 8, 0.1)]
    # ped1.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
    # peds.append(ped1)

    ped2 = Ped()
    ped2.id = 2
    ped2.pos = Point(1, 1, 0)
    ped2.type = "forklift"
    ped2.number_of_peds = 1
    ped2.vmax = 2.0
    ped2.force_factor_desired = 1.0
    ped2.force_factor_obstacle = 1.0
    ped2.force_factor_social = 1.0
    ped2.waypoint_mode = 1
    # ped2.waypoints = [Point(8, 3, 0.1), Point(3, 8, 0.1), Point(8, 8, 0.1), Point(3, 3, 0.1)]
    ped2.waypoints = [Point(9, 3, 0.1), Point(9, 11, 0.1)]
    ped2.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "big_forklift.model.yaml")
    peds.append(ped2)

    response = respawn_ped_srv.call(peds)
    print("successfully spawned peds" if response.success else "failed")


    respawn_interactive_obstacles_service_name = "pedsim_simulator/respawn_interactive_obstacles"
    rospy.wait_for_service(respawn_interactive_obstacles_service_name, 6.0)
    spawn_interactive_obstacles_client = rospy.ServiceProxy(respawn_interactive_obstacles_service_name, SpawnInteractiveObstacles)

    obstacle = InteractiveObstacle()
    obstacle.type = "shelf"
    obstacle.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "obstacles", "shelf.yaml")
    obstacle.pose.orientation.w = 1
    obstacle.pose.position = Point(10.0, 10.0, 0)
    obstacle.interaction_radius = 4

    obstacle2 = copy.deepcopy(obstacle)
    obstacle.pose.position = Point(10.0, 3.0, 0)

    spawn_interactive_obstacles_client.call([obstacle, obstacle2])


    # time.sleep(8)
    # response = respawn_ped_srv.call(peds)

    # time.sleep(8)
    # spawn_interactive_obstacles_client.call([obstacle, obstacle2])



def example2():
    rospack = rospkg.RosPack()
    respawn_peds_service_name = "pedsim_simulator/respawn_peds"
    rospy.wait_for_service(respawn_peds_service_name, 6.0)
    time.sleep(4)
    respawn_ped_srv = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)

    for _ in range(1):
        peds = []

        ped1 = Ped()
        ped1.id = 2
        ped1.pos = Point(3, 10, 0)
        ped1.type = "adult"
        ped1.number_of_peds = 2
        ped1.max_talking_distance = 2.0
        ped1.chatting_probability = 0.7
        ped1.group_talking_probability = 0.0
        ped1.group_talking_base_time = 20.0
        ped1.tell_story_probability = 0.0
        ped1.tell_story_base_time = 20.0
        ped1.talking_and_walking_probability = 0.0
        ped1.talking_and_walking_base_time = 20.0
        ped1.vmax = 2.0
        ped1.force_factor_desired = 1.0
        ped1.force_factor_obstacle = 1.0
        ped1.force_factor_social = 10.0
        ped1.waypoint_mode = 0
        ped1.waypoints = [Point(10, 3, 0.1), Point(7, 15, 0.1)]
        ped1.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
        peds.append(ped1)

        # ped2 = copy.copy(ped1)
        # # ped2 = ped1
        # ped2.pos = Point(6, 2, 0.1)
        # ped2.waypoints = [Point(6, 6, 0.1)]
        # peds.append(ped2)


        response = respawn_ped_srv.call(peds)
        print("successfully spawned peds" if response.success else "failed")


        # time.sleep(1)


        # peds = []

        # ped2 = Ped()
        # ped2.id = 2
        # ped2.pos = Point(1, 3, 0)
        # ped2.type = 0
        # ped2.number_of_peds = 8
        # ped2.vmax = 1.0
        # ped2.force_factor_desired = 1.0
        # ped2.force_factor_obstacle = 1.0
        # ped2.force_factor_social = 1.0
        # ped2.waypoint_mode = 0
        # ped2.waypoints = [Point(3, 3, 0.1), Point(3, 8, 0.1), Point(8, 8, 0.1), Point(8, 3, 0.1)]
        # ped2.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
        # peds.append(ped2)

        # response = respawn_ped_srv.call(peds)
        # print("successfully spawned peds" if response.success else "failed")


        # time.sleep(1)


    # peds = []

    # ped3 = Ped()
    # ped3.id = 2
    # ped3.pos = Point(1, 3, 0)
    # ped3.type = 0
    # ped3.number_of_peds = 7
    # ped3.vmax = 1.0
    # ped3.force_factor_desired = 1.0
    # ped3.force_factor_obstacle = 1.0
    # ped3.force_factor_social = 1.0
    # ped3.waypoint_mode = 0
    # ped3.waypoints = [Point(3, 3, 0.1), Point(3, 8, 0.1), Point(8, 8, 0.1), Point(8, 3, 0.1)]
    # ped3.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
    # peds.append(ped3)

    # response = respawn_ped_srv.call(peds)
    # print("successfully spawned peds" if response.success else "failed")

def test_interaction():
    rospack = rospkg.RosPack()
    respawn_peds_service_name = "pedsim_simulator/respawn_peds"
    rospy.wait_for_service(respawn_peds_service_name, 6.0)
    respawn_ped_srv = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)

    time.sleep(1)

    peds = []

    ped1 = Ped()
    ped1.id = 2
    ped1.pos = Point(8, 3, 0)
    ped1.type = "adult"
    ped1.number_of_peds = 1
    ped1.max_talking_distance = 2.0
    ped1.chatting_probability = 0.0
    ped1.group_talking_probability = 0.0
    ped1.group_talking_base_time = 20.0
    ped1.tell_story_probability = 0.0
    ped1.tell_story_base_time = 20.0
    ped1.talking_and_walking_probability = 0.0
    ped1.talking_and_walking_base_time = 20.0

    ped1.requesting_service_base_time = 30.0
    ped1.receiving_service_base_time = 20.0
    ped1.requesting_service_probability = 0.1

    ped1.vmax = 2.0
    ped1.force_factor_desired = 1.0
    ped1.force_factor_obstacle = 1.0
    ped1.force_factor_social = 10.0
    ped1.waypoint_mode = 0
    ped1.waypoints = [Point(10, 3, 0.1), Point(10, 10, 0.1)]
    ped1.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
    peds.append(ped1)


    ped2 = copy.deepcopy(ped1)
    ped2.pos = Point(6, 10, 0.1)
    ped2.type = "servicerobot"
    ped2.number_of_peds = 1
    ped2.max_servicing_radius = 10.0
    ped2.waypoints = [Point(8, 10, 0.1), Point(8, 3, 0.1)]
    ped2.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "servicerobot.model.yaml")
    peds.append(ped2)

    response = respawn_ped_srv.call(peds)
    print("successfully spawned peds" if response.success else "failed")

if __name__ == '__main__':
    test_interaction()