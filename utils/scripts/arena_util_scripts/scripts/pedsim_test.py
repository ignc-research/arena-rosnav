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

def get_yaml_path_from_type(ped_type):
    rospack = rospkg.RosPack()
    if ped_type == "adult":
        return os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
    elif ped_type == "child":
        return os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
    elif ped_type == "elder":
        return os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
    elif ped_type == "forklift":
        return os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "big_forklift.model.yaml")
    elif ped_type == "servicerobot":
        return os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "servicerobot.model.yaml")
    raise Exception("unknown ped type:", ped_type)

def get_default_ped(id, ped_type, yaml_path, pos, waypoints):
    ped = Ped()
    ped.id = id
    ped.pos = pos
    ped.type = ped_type
    ped.yaml_file = yaml_path
    ped.number_of_peds = 1
    ped.vmax = 1.0

    ped.chatting_probability = 0.01
    ped.tell_story_probability = 0.01
    ped.group_talking_probability = 0.01
    ped.talking_and_walking_probability = 0.01
    ped.requesting_service_probability = 0.01

    ped.talking_base_time = 10.0
    ped.tell_story_base_time = 10.0
    ped.group_talking_base_time = 10.0
    ped.talking_and_walking_base_time = 6.0
    ped.receiving_service_base_time = 20.0
    ped.requesting_service_base_time = 30.0

    ped.max_talking_distance = 2.0
    ped.max_servicing_radius = 5.0

    ped.force_factor_desired = 1.0
    ped.force_factor_obstacle = 1.0
    ped.force_factor_social = 1.0

    ped.waypoints = waypoints
    ped.waypoint_mode = 0
    return ped

def social_force_test():
    time.sleep(2)
    respawn_peds_service_name = "pedsim_simulator/respawn_peds"
    rospy.wait_for_service(respawn_peds_service_name, 6.0)
    respawn_ped_srv = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)

    peds = []

    ped = get_default_ped(
            id = 1,
            ped_type = "adult",
            yaml_path = get_yaml_path_from_type("adult"),
            pos = Point(10, 3, 0.1),
            waypoints = [Point(10, 10, 0.1), Point(10, 3, 0.1)]
        )
    ped.chatting_probability = 0
    ped.talking_and_walking_probability = 0
    ped.requesting_service_probability = 0
    ped.force_factor_social = 20.0
    peds.append(ped)

    ped = get_default_ped(
            id = 2,
            ped_type = "forklift",
            yaml_path = get_yaml_path_from_type("forklift"),
            pos = Point(10, 10, 0.1),
            waypoints = [Point(10, 3, 0.1), Point(10, 10, 0.1)]
        )
    ped.chatting_probability = 0
    ped.talking_and_walking_probability = 0
    ped.requesting_service_probability = 0
    ped.force_factor_social = 20.0
    peds.append(ped)

    response = respawn_ped_srv.call(peds)
    print("successfully spawned peds" if response.success else "failed")

def crowd_test():
    time.sleep(2)
    respawn_peds_service_name = "pedsim_simulator/respawn_peds"
    rospy.wait_for_service(respawn_peds_service_name, 6.0)
    respawn_ped_srv = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)

    peds = []

    ped = get_default_ped(
            id = 1,
            ped_type = "adult",
            yaml_path = get_yaml_path_from_type("adult"),
            pos = Point(5, 5, 0.1),
            waypoints = [Point(10, 10, 0.1), Point(10, 3, 0.1), Point(3, 10, 0.1), Point(3, 3, 0.1)]
        )
    ped.number_of_peds = 15
    ped.waypoint_mode = 1
    ped.chatting_probability = 0
    ped.talking_and_walking_probability = 0
    ped.tell_story_probability = 0
    ped.group_talking_probability = 0
    ped.requesting_service_probability = 0
    ped.force_factor_social = 3.0
    peds.append(ped)

    response = respawn_ped_srv.call(peds)
    print("successfully spawned peds" if response.success else "failed")

def shelves_test():
    time.sleep(2)
    rospack = rospkg.RosPack()
    respawn_peds_service_name = "pedsim_simulator/respawn_peds"
    rospy.wait_for_service(respawn_peds_service_name, 6.0)
    respawn_ped_srv = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)

    peds = []

    # ped1 = Ped()
    # ped1.id = 2
    # ped1.pos = Point(1, 10, 0)
    # ped1.type = "adult"
    # ped1.number_of_peds = 3
    # ped1.max_talking_distance = 3
    # ped1.chatting_probability = 0.0
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
    ped2.pos = Point(-1, 3, 0)
    ped2.type = "forklift"
    ped2.number_of_peds = 1
    ped2.vmax = 2.0
    ped2.force_factor_desired = 1.0
    ped2.force_factor_obstacle = 1.0
    ped2.force_factor_social = 1.0
    ped2.waypoint_mode = 0
    # ped2.waypoints = [Point(8, 3, 0.1), Point(3, 8, 0.1), Point(8, 8, 0.1), Point(3, 3, 0.1)]
    ped2.waypoints = [Point(7, 3, 0.1), Point(9, 11, 0.1), Point(5, 5, 0.1)]
    ped2.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "big_forklift.model.yaml")
    peds.append(ped2)

    response = respawn_ped_srv.call(peds)
    print("successfully spawned peds" if response.success else "failed")


    respawn_interactive_obstacles_service_name = "pedsim_simulator/respawn_interactive_obstacles"
    rospy.wait_for_service(respawn_interactive_obstacles_service_name, 6.0)
    respawn_interactive_obstacles_client = rospy.ServiceProxy(respawn_interactive_obstacles_service_name, SpawnInteractiveObstacles)

    obstacle = InteractiveObstacle()
    obstacle.name = "wp1"
    obstacle.type = "shelf"
    obstacle.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "obstacles", "shelf.yaml")
    obstacle.pose.orientation.w = 1
    obstacle.pose.position = Point(10.0, 10.0, 0)
    obstacle.interaction_radius = 5

    obstacle2 = copy.deepcopy(obstacle)
    obstacle2.name = "wp2"
    obstacle.pose.position = Point(10.0, 3.0, 0)

    respawn_interactive_obstacles_client.call([obstacle, obstacle2])


    # time.sleep(8)
    # response = respawn_ped_srv.call(peds)

    # time.sleep(8)
    # respawn_interactive_obstacles_client.call([obstacle, obstacle2])



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

def interaction_test():
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
    ped1.receiving_service_base_time = 10.0
    ped1.requesting_service_probability = 0.05

    ped1.vmax = 2.0
    ped1.force_factor_desired = 1.0
    ped1.force_factor_obstacle = 1.0
    ped1.force_factor_social = 1.0
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

def follow_robot_test():
    respawn_peds_service_name = "pedsim_simulator/respawn_peds"
    rospy.wait_for_service(respawn_peds_service_name, 6.0)
    respawn_ped_srv = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)

    time.sleep(1)

    peds = []
    ped = get_default_ped(
            id = 1,
            ped_type = "adult",
            yaml_path = get_yaml_path_from_type("adult"),
            pos = Point(5, 5, 0.1),
            waypoints = [Point(10, 10, 0.1), Point(10, 3, 0.1), Point(3, 10, 0.1), Point(3, 3, 0.1)]
        )
    ped.requesting_service_probability = 0.0
    peds.append(ped)
    response = respawn_ped_srv.call(peds)
    print("successfully spawned peds" if response.success else "failed")

if __name__ == '__main__':
    # shelves_test()
    # interaction_test()
    # social_force_test()
    # crowd_test()
    follow_robot_test()
