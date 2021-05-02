#!/usr/bin/env python
import rospy
import os
import rospkg
import sys
import time
from std_srvs.srv import SetBool
from pedsim_srvs.srv import SpawnPeds
from pedsim_msgs.msg import Ped
from geometry_msgs.msg import Point

def example1():
    rospack = rospkg.RosPack()
    spawn_peds_service_name = "pedsim_simulator/spawn_peds"
    rospy.wait_for_service(spawn_peds_service_name, 6.0)
    spawn_ped_srv = rospy.ServiceProxy(spawn_peds_service_name, SpawnPeds)

    peds = []

    ped1 = Ped()
    ped1.id = 2
    ped1.pos = Point(6, 3, 0)
    ped1.type = 0
    ped1.number_of_peds = 5
    ped1.vmax = 1.0
    ped1.force_factor_desired = 1.0
    ped1.force_factor_obstacle = 1.0
    ped1.force_factor_social = 1.0
    ped1.waypoint_mode = 1
    ped1.waypoints = [Point(7, 2, 0.1), Point(7, 8, 0.1), Point(3, 9, 0.1)]
    ped1.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
    peds.append(ped1)

    ped2 = Ped()
    ped2.id = 2
    ped2.pos = Point(1, 3, 0)
    ped2.type = 0
    ped2.number_of_peds = 3
    ped2.vmax = 1.0
    ped2.force_factor_desired = 1.0
    ped2.force_factor_obstacle = 1.0
    ped2.force_factor_social = 1.0
    ped2.waypoint_mode = 1
    ped2.waypoints = [Point(3, 3, 0.1), Point(3, 8, 0.1), Point(8, 8, 0.1), Point(8, 3, 0.1)]
    ped2.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
    peds.append(ped2)

    response = spawn_ped_srv.call(peds)
    print("successfully spawned peds" if response.success else "failed")

    time.sleep(5)

    remove_all_peds_service_name = "pedsim_simulator/remove_all_peds"
    rospy.wait_for_service(remove_all_peds_service_name, 6.0)
    remove_all_peds_srv = rospy.ServiceProxy(remove_all_peds_service_name, SetBool)
    response = remove_all_peds_srv.call()
    print("remove all peds: ", response.success)


def example2():
    rospack = rospkg.RosPack()
    respawn_peds_service_name = "pedsim_simulator/respawn_peds"
    rospy.wait_for_service(respawn_peds_service_name, 6.0)
    respawn_ped_srv = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)

    for _ in range(20):
        peds = []

        ped1 = Ped()
        ped1.id = 2
        ped1.pos = Point(6, 3, 0)
        ped1.type = 0
        ped1.number_of_peds = 7
        ped1.vmax = 1.0
        ped1.force_factor_desired = 1.0
        ped1.force_factor_obstacle = 1.0
        ped1.force_factor_social = 10.0
        ped1.waypoint_mode = 0
        ped1.waypoints = [Point(7, 2, 0.1), Point(7, 8, 0.1), Point(3, 9, 0.1)]
        ped1.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
        peds.append(ped1)


        response = respawn_ped_srv.call(peds)
        print("successfully spawned peds" if response.success else "failed")


        time.sleep(1)


        peds = []

        ped2 = Ped()
        ped2.id = 2
        ped2.pos = Point(1, 3, 0)
        ped2.type = 0
        ped2.number_of_peds = 8
        ped2.vmax = 1.0
        ped2.force_factor_desired = 1.0
        ped2.force_factor_obstacle = 1.0
        ped2.force_factor_social = 1.0
        ped2.waypoint_mode = 0
        ped2.waypoints = [Point(3, 3, 0.1), Point(3, 8, 0.1), Point(8, 8, 0.1), Point(8, 3, 0.1)]
        ped2.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
        peds.append(ped2)

        response = respawn_ped_srv.call(peds)
        print("successfully spawned peds" if response.success else "failed")


        time.sleep(1)


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


if __name__ == '__main__':
    example2()