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

rospack = rospkg.RosPack()

pedsim_types_model_paths = {
    "adult": os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml"),
    "elder": os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml"),
    "child": os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml"),
    "forklift": os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "big_forklift.model.yaml"),
}

spawn_peds_service_name = "pedsim_simulator/spawn_peds"
rospy.wait_for_service(spawn_peds_service_name, 6.0)
spawn_ped_srv = rospy.ServiceProxy(spawn_peds_service_name, SpawnPeds)

peds = []

ped1 = Ped()
ped1.id = 2
ped1.pos = Point(1, 1, 0)
ped1.type = "adult"
ped1.number_of_peds = 2
ped1.vmax = 1.5
ped1.force_factor_desired = 1.0
ped1.force_factor_obstacle = 1.0
ped1.force_factor_social = 2.0
ped1.chatting_probability = 0.01
ped1.tell_story_probability = 0.01
ped1.group_talking_probability = 0.01
ped1.talking_and_walking_probability = 0.01
ped1.max_talking_distance = 2.0
ped1.talking_base_time = 6.0
ped1.tell_story_base_time = 10.0
ped1.group_talking_base_time = 10.0
ped1.talking_and_walking_base_time = 10.0
ped1.waypoint_mode = 1
ped1.waypoints = [Point(7, 2, 0.1), Point(7, 8, 0.1), Point(3, 9, 0.1)]
ped1.yaml_file = pedsim_types_model_paths["adult"]
peds.append(ped1)

ped2 = Ped()
ped2.id = 2
ped2.pos = Point(6, 3, 0)
ped2.type = "adult"
ped2.number_of_peds = 6
ped2.vmax = 2.0
ped2.force_factor_desired = 2.0
ped2.force_factor_obstacle = 1.0
ped2.force_factor_social = 2.0
ped2.chatting_probability = 0.01
ped2.tell_story_probability = 0.01
ped2.group_talking_probability = 0.5
ped2.talking_and_walking_probability = 0.1
ped2.max_talking_distance = 2.0
ped2.talking_base_time = 6.0
ped2.tell_story_base_time = 10.0
ped2.group_talking_base_time = 10.0
ped2.talking_and_walking_base_time = 10.0
ped2.waypoint_mode = 0
ped2.waypoints = [Point(7, 2, 0.1), Point(7, 8, 0.1), Point(3, 9, 0.1)]
ped2.yaml_file = pedsim_types_model_paths["adult"]
peds.append(ped2)

ped3 = Ped()
ped3.id = 2
ped3.pos = Point(8, 7, 0)
ped3.type = "forklift"
ped3.number_of_peds = 2
ped3.vmax = 4.0
ped3.force_factor_desired = 1.0
ped3.force_factor_obstacle = 1.0
ped3.force_factor_social = 2.0
ped3.waypoint_mode = 1
ped3.waypoints = [Point(3, 15, 0.1), Point(9, 9, 0.1), Point(20, 9, 0.1), Point(17, 7, 0.1)]
ped3.yaml_file = pedsim_types_model_paths["forklift"]
peds.append(ped3)

response = spawn_ped_srv.call(peds)
print("successfully spawned peds" if response.success else "failed to spawn peds")

