#!/usr/bin/env python
import rospy
import os
import sys
import time
from pedsim_srvs.srv import SpawnInteractiveObstacles
from pedsim_srvs.srv import SpawnPeds
from pedsim_msgs.msg import Ped
from geometry_msgs.msg import Point
from pedsim_test import get_default_ped, get_yaml_path_from_type, get_shelf

spawn_peds_service_name = "pedsim_simulator/spawn_peds"
rospy.wait_for_service(spawn_peds_service_name, 6.0)
spawn_ped_srv = rospy.ServiceProxy(spawn_peds_service_name, SpawnPeds)

peds = []

# humans
ped = get_default_ped(
        id = 1,
        ped_type = "adult",
        yaml_path = get_yaml_path_from_type("adult"),
        pos = Point(0, 3, 0.1),
        waypoints = [Point(0, 6, 0.1), Point(8, 10, 0.1), Point(0, 13, 0.1)]
    )
ped.number_of_peds = 5
ped.requesting_service_probability = 0.005
ped.requesting_follower_probability = 0
ped.requesting_guide_probability = 0
ped.group_talking_base_time = 20.0
peds.append(ped)

# forklift
ped = get_default_ped(
        id = 2,
        ped_type = "forklift",
        yaml_path = get_yaml_path_from_type("forklift"),
        pos = Point(18, 1, 0.1),
        waypoints = [Point(18, 4, 0.1), Point(18, 12, 0.1)]
    )
ped.vmax = 2.0
# ped.force_factor_obstacle = 1.5
peds.append(ped)

# service robot
ped = get_default_ped(
        id = 3,
        ped_type = "servicerobot",
        yaml_path = get_yaml_path_from_type("servicerobot"),
        pos = Point(5, 2, 0.1),
        waypoints = [Point(5, 5, 0.1), Point(5, 16, 0.1)]
    )
peds.append(ped)

response = spawn_ped_srv.call(peds)
print("successfully spawned peds" if response.success else "failed to spawn peds")


# spawn shelves for the forklift
respawn_interactive_obstacles_service_name = "pedsim_simulator/respawn_interactive_obstacles"
rospy.wait_for_service(respawn_interactive_obstacles_service_name, 6.0)
respawn_interactive_obstacles_client = rospy.ServiceProxy(respawn_interactive_obstacles_service_name, SpawnInteractiveObstacles)

shelf1 = get_shelf("wp1", Point(16, 4, 0.3))
shelf2 = get_shelf("wp2", Point(16, 12, 0.3))

respawn_interactive_obstacles_client.call([shelf1, shelf2])

