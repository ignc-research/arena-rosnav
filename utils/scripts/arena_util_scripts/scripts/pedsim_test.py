#!/usr/bin/env python
import numpy as np
import rospy
import os
import rospkg
import sys
import time
import copy
from typing import List
from std_srvs.srv import Trigger
from pedsim_srvs.srv import SpawnPeds
from pedsim_srvs.srv import SpawnInteractiveObstacles
from pedsim_srvs.srv import SpawnObstacle
from pedsim_msgs.msg import Ped
from pedsim_msgs.msg import InteractiveObstacle
from pedsim_msgs.msg import LineObstacles
from pedsim_msgs.msg import LineObstacle
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

class PedsimManager():
    def __init__(self):
        # spawn peds
        spawn_peds_service_name = "pedsim_simulator/spawn_peds"
        rospy.wait_for_service(spawn_peds_service_name, 6.0)
        self.spawn_peds_client = rospy.ServiceProxy(spawn_peds_service_name, SpawnPeds)
        # respawn peds
        respawn_peds_service_name = "pedsim_simulator/respawn_peds"
        rospy.wait_for_service(respawn_peds_service_name, 6.0)
        self.respawn_peds_client = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)
        # spawn interactive obstacles
        pawn_interactive_obstacles_service_name = "pedsim_simulator/spawn_interactive_obstacles"
        rospy.wait_for_service(pawn_interactive_obstacles_service_name, 6.0)
        self.spawn_interactive_obstacles_client = rospy.ServiceProxy(pawn_interactive_obstacles_service_name, SpawnInteractiveObstacles)
        # respawn interactive obstacles
        respawn_interactive_obstacles_service_name = "pedsim_simulator/respawn_interactive_obstacles"
        rospy.wait_for_service(respawn_interactive_obstacles_service_name, 6.0)
        self.respawn_interactive_obstacles_client = rospy.ServiceProxy(respawn_interactive_obstacles_service_name, SpawnInteractiveObstacles)

    def spawnPeds(self, peds: List[Ped]):
        res = self.spawn_peds_client.call(peds)
        print(res)

    def respawnPeds(self, peds: List[Ped]):
        res = self.respawn_peds_client.call(peds)
        print(res)

    def spawnInteractiveObstacles(self, obstacles: List[InteractiveObstacle]):
        res = self.spawn_interactive_obstacles_client.call(obstacles)
        print(res)

    def respawnInteractiveObstacles(self, obstacles: List[InteractiveObstacle]):
        res = self.respawn_interactive_obstacles_client.call(obstacles)
        print(res)


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
    elif ped_type == "shelf":
        return os.path.join(rospack.get_path("simulator_setup"), "obstacles", "shelf.yaml")
    raise Exception("unknown ped type:", ped_type)

def get_default_ped(id, ped_type, yaml_path, pos, waypoints) -> Ped:
    ped = Ped()
    ped.id = id
    ped.pos = pos
    ped.type = ped_type
    ped.yaml_file = yaml_path
    ped.number_of_peds = 1
    ped.vmax = 1.0

    ped.start_up_mode = "default"
    ped.wait_time = 0
    ped.trigger_zone_radius = 0

    ped.chatting_probability = 0.01
    ped.tell_story_probability = 0.01
    ped.group_talking_probability = 0.01
    ped.talking_and_walking_probability = 0.01
    ped.requesting_service_probability = 0.01
    ped.requesting_guide_probability = 0.01
    ped.requesting_follower_probability = 0.01

    ped.talking_base_time = 10.0
    ped.tell_story_base_time = 10.0
    ped.group_talking_base_time = 10.0
    ped.talking_and_walking_base_time = 6.0
    ped.receiving_service_base_time = 20.0
    ped.requesting_service_base_time = 30.0

    ped.max_talking_distance = 2.0
    ped.max_servicing_radius = 10.0

    ped.force_factor_desired = 1.0
    ped.force_factor_obstacle = 1.0
    ped.force_factor_social = 5.0
    ped.force_factor_robot = 0.0

    ped.waypoints = waypoints
    ped.waypoint_mode = 0
    return ped

def get_only_moving_ped(id, ped_type, yaml_path, pos, waypoints) -> Ped:
    ped = get_default_ped(id, ped_type, yaml_path, pos, waypoints)
    ped.chatting_probability = 0
    ped.tell_story_probability = 0
    ped.group_talking_probability = 0
    ped.talking_and_walking_probability = 0
    ped.requesting_service_probability = 0
    ped.requesting_guide_probability = 0
    ped.requesting_follower_probability = 0
    return ped

def get_shelf(name: str, position: Point) -> InteractiveObstacle:
    obstacle = InteractiveObstacle()
    obstacle.name = name
    obstacle.type = "shelf"
    obstacle.yaml_path = get_yaml_path_from_type("shelf")
    obstacle.pose.orientation.w = 1
    obstacle.pose.position = position
    obstacle.interaction_radius = 5
    return obstacle

def get_random_shelf():
    msg = InteractiveObstacle()   
    msg.pose = Pose()
    msg.pose.position.x = np.random.uniform(low=-2.0, high=15.0)
    msg.pose.position.y = np.random.uniform(low=-2.0, high=15.0)
    msg.pose.position.z = np.random.uniform(low=-2.0, high=15.0)
    msg.interaction_radius = 4.0
    msg.type = "shelf"
    msg.yaml_path = get_yaml_path_from_type("shelf")
    return msg

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
    ped.requesting_guide_probability = 0
    ped.force_factor_social = 20.0
    peds.append(ped)

    ped = get_default_ped(
            id = 2,
            ped_type = "adult",
            yaml_path = get_yaml_path_from_type("adult"),
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
    ped.number_of_peds = 8
    ped.waypoint_mode = 1
    ped.chatting_probability = 0
    ped.talking_and_walking_probability = 0
    ped.tell_story_probability = 0
    ped.group_talking_probability = 0.01
    ped.group_talking_base_time = 300.0
    ped.requesting_service_probability = 0
    ped.requesting_guide_probability = 0
    ped.force_factor_social = 1.0
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
    obstacle.pose.position = Point(10.0, 14.0, 0)
    obstacle.interaction_radius = 5

    obstacle2 = copy.deepcopy(obstacle)
    obstacle2.name = "wp2"
    obstacle.pose.position = Point(10.0, 3.0, 0)

    respawn_interactive_obstacles_client.call([obstacle, obstacle2])

def respawn_shelves_test():
    # setup respawn_interactive_obstacles
    respawn_interactive_obstacles_service_name = "pedsim_simulator/respawn_interactive_obstacles"
    rospy.wait_for_service(respawn_interactive_obstacles_service_name, 6.0)
    respawn_interactive_obstacles_client = rospy.ServiceProxy(respawn_interactive_obstacles_service_name, SpawnInteractiveObstacles)

    # setup remove_all_interactive_obstacles
    remove_all_interactive_obstacles_service_name = "pedsim_simulator/remove_all_interactive_obstacles"
    rospy.wait_for_service(remove_all_interactive_obstacles_service_name, 6.0)
    remove_all_interactive_obstacles_client = rospy.ServiceProxy(remove_all_interactive_obstacles_service_name, Trigger)

    for i in range(10):
        num = np.random.randint(low=1, high=10)
        msgs = [get_random_shelf() for j in range(num)]
        remove_all_interactive_obstacles_client()
        respawn_interactive_obstacles_client(msgs)
        time.sleep(10)

def example2():
    rospack = rospkg.RosPack()
    respawn_peds_service_name = "pedsim_simulator/respawn_peds"
    rospy.wait_for_service(respawn_peds_service_name, 6.0)
    time.sleep(4)
    respawn_ped_srv = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)

    for _ in range(2):
        peds = []

        ped1 = Ped()
        ped1.id = 2
        ped1.pos = Point(3, 10, 0)
        ped1.type = "adult"
        ped1.number_of_peds = 2
        ped1.max_talking_distance = 2.0
        ped1.chatting_probability = 0.0
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


        time.sleep(5)


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

def service_robot_test():
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
    ped1.requesting_service_probability = 0.99

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
    ped2.waypoints = [Point(6, 10, 0.1), Point(6, 3, 0.1)]
    ped2.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "servicerobot.model.yaml")
    peds.append(ped2)

    # ped = copy.deepcopy(ped1)
    # ped.pos = Point(1, 10, 0.1)
    # ped.type = "servicerobot"
    # ped.number_of_peds = 1
    # ped.max_servicing_radius = 10.0
    # ped.waypoints = [Point(6, 10, 0.1), Point(6, 3, 0.1)]
    # ped.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "servicerobot.model.yaml")
    # peds.append(ped)

    response = respawn_ped_srv.call(peds)



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
    ped.requesting_guide_probability = 0.05
    peds.append(ped)
    response = respawn_ped_srv.call(peds)
    print("successfully spawned peds" if response.success else "failed")

def get_circle(center, radius):
    obstacles = []
    num_points = 12
    delta_angle = 2 * np.pi / float(num_points)
    rotation = np.array([[np.cos(delta_angle), -np.sin(delta_angle)],
                         [np.sin(delta_angle), np.cos(delta_angle)]])
    points = []
    for i in range(num_points):
        base_vector = np.array([radius, 0])
        # rotate i times
        for _ in range(i):
            base_vector = rotation.dot(base_vector)
        points.append(center + base_vector)

    for i in range(num_points):
        obstacle = LineObstacle()
        start = points[i]
        end = points[(i+1) % num_points]
        obstacle.start = Point(start[0], start[1], 0)
        obstacle.end = Point(end[0], end[1], 0)
        obstacles.append(obstacle)

    return obstacles

def obstacle_force_test():
    # spawn ped service
    spawn_peds_service_name = "pedsim_simulator/spawn_peds"
    rospy.wait_for_service(spawn_peds_service_name, 6.0)
    spawn_peds_client = rospy.ServiceProxy(spawn_peds_service_name, SpawnPeds)

    # add wall service
    add_obstacles_service_name = "pedsim_simulator/add_obstacle"
    rospy.wait_for_service(add_obstacles_service_name, 6.0)
    add_obstacles_client = rospy.ServiceProxy(add_obstacles_service_name, SpawnObstacle)

    ## normal wall
    ped = get_default_ped(
            id = 1,
            ped_type = "adult",
            yaml_path = get_yaml_path_from_type("adult"),
            pos = Point(4, 2, 0.1),
            waypoints = [Point(4, 2, 0.1), Point(12, 2, 0.1)]
        )
    ped.requesting_guide_probability = 0
    ped.requesting_follower_probability = 0
    ped.force_factor_obstacle = 2

    spawn_peds_client.call([ped])

    obstacles = LineObstacles()
    obstacle = LineObstacle()
    obstacle.start = Point(9, 4, 0)
    obstacle.end = Point(9, 1, 0)
    obstacles.obstacles.append(obstacle)

    add_obstacles_client.call(obstacles)

    ## circular wall
    ped = get_default_ped(
            id = 1,
            ped_type = "adult",
            yaml_path = get_yaml_path_from_type("adult"),
            pos = Point(4, 10, 0.1),
            waypoints = [Point(4, 10.2, 0.1), Point(12, 10.2, 0.1)]
        )
    ped.requesting_guide_probability = 0
    ped.requesting_follower_probability = 0
    ped.force_factor_obstacle = 2

    spawn_peds_client.call([ped])

    obstacles = LineObstacles()
    obstacle = LineObstacle()
    obstacle.start = Point(9, 12, 0)
    obstacle.end = Point(9, 8, 0)
    obstacles.obstacles.append(obstacle)
    obstacles.obstacles += get_circle(center=np.array([9, 10]), radius=2.0)

    add_obstacles_client.call(obstacles)

def guide_to_goal_test():
    # spawn ped service
    spawn_peds_service_name = "pedsim_simulator/spawn_peds"
    rospy.wait_for_service(spawn_peds_service_name, 6.0)
    spawn_peds_client = rospy.ServiceProxy(spawn_peds_service_name, SpawnPeds)

    ped = get_only_moving_ped(
            id = 1,
            ped_type = "adult",
            yaml_path = get_yaml_path_from_type("adult"),
            pos = Point(2, 2, 0.1),
            waypoints = [Point(4, 2, 0.1), Point(12, 2, 0.1)]
        )
    ped.number_of_peds = 1
    ped.requesting_follower_probability = 0.9
    ped.force_factor_robot = 10.0

    spawn_peds_client.call([ped])

def running_test():
    # spawn ped service
    spawn_peds_service_name = "pedsim_simulator/spawn_peds"
    rospy.wait_for_service(spawn_peds_service_name, 6.0)
    spawn_peds_client = rospy.ServiceProxy(spawn_peds_service_name, SpawnPeds)

    ped = get_default_ped(
            id = 1,
            ped_type = "adult",
            yaml_path = get_yaml_path_from_type("adult"),
            pos = Point(2, 2, 0.1),
            # waypoints = [Point(4, 2, 0.1), Point(12, 2, 0.1)]
            waypoints = [Point(4, 2, 0.1), Point(12, 2, 0.1), Point(10, 7, 0.1)]
        )
    ped.number_of_peds = 1
    ped.vmax = 3.0
    ped.requesting_guide_probability = 0
    ped.requesting_service_probability = 0
    ped.requesting_follower_probability = 0
    ped.chatting_probability = 0
    ped.talking_and_walking_probability = 0
    ped.force_factor_robot = 10.0

    spawn_peds_client.call([ped])

def wait_timer_test():
    pmanager = PedsimManager()

    ped = get_only_moving_ped(
            id = 1,
            ped_type = "adult",
            yaml_path = get_yaml_path_from_type("adult"),
            pos = Point(2, 2, 0.1),
            waypoints = [Point(4, 2, 0.1), Point(12, 2, 0.1), Point(10, 7, 0.1)]
        )
    ped.start_up_mode = "wait_timer"
    ped.wait_time = 10

    pmanager.spawnPeds([ped])

def trigger_zone_test():
    pmanager = PedsimManager()

    ped = get_only_moving_ped(
            id = 1,
            ped_type = "adult",
            yaml_path = get_yaml_path_from_type("adult"),
            pos = Point(5, 4, 0.1),
            waypoints = [Point(4, 2, 0.1), Point(12, 2, 0.1), Point(10, 7, 0.1)]
        )
    ped.start_up_mode = "trigger_zone"
    ped.trigger_zone_radius = 3

    pmanager.spawnPeds([ped])

if __name__ == '__main__':
    # shelves_test()
    # service_robot_test()
    # social_force_test()
    # crowd_test()
    # follow_robot_test()
    # example2()
    # respawn_shelves_test()
    # obstacle_force_test()
    guide_to_goal_test()
    # running_test()
    # wait_timer_test()
    # trigger_zone_test()
