#!/usr/bin/env python3
import random

import nav_msgs.srv
import rospkg
import rospy
from map_generator import *
from nav_msgs.msg import OccupancyGrid

from simulator_setup.srv import *


class MapGenerator:
    def __init__(self):
        # initial value for scenario number
        self.nr = -1

        # general map parameter
        self.height = 250
        self.width = 250

        self.resolution = 0.11

        # indoor map parameter
        self.corridor_radius = 8
        self.iterations = 50
        self.room_number = 3
        self.room_width = 75
        self.room_height = 70
        self.no_overlap = True

        # outdoor map parameter
        self.obstacle_number = 15
        self.obstacle_extra_radius = 6

        # initialize occupancy grid
        self.occupancy_grid = OccupancyGrid()
        self.ns = rospy.get_param("~ns")
        self.map_type = rospy.get_param("~map_type")
        self.indoor_prob = rospy.get_param("~indoor_prob")

        # self.generate_initial_map() # initial random map generation (before first episode)
        rospy.Subscriber("/" + self.ns + '/map', OccupancyGrid, self.get_occupancy_grid)
        # generate new random map for the next episode when entering new episode
        rospy.Service("/" + self.ns + '/new_map', GetMapWithSeed, self.new_episode_callback)

        self.mappub = rospy.Publisher("/" + self.ns + '/map', OccupancyGrid, queue_size=1)

        # initialize yaml files
        map_dir = os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'maps')
        create_yaml_files('random_map', map_dir, self.resolution, self.ns)

    # a bit cheating: copy OccupancyGrid meta data from map_server of initial map
    def get_occupancy_grid(self, occgrid_msg):
        self.occupancy_grid = occgrid_msg

    def generate_initial_map(self):  # generate random map png in random_map directory
        map = create_random_map(
            height=self.height,
            width=self.width,
            corridor_radius=self.corridor_radius,
            iterations=self.iterations,
            obstacle_number=self.obstacle_number,
            obstacle_extra_radius=self.obstacle_extra_radius,
            room_number=self.room_number,
            room_width=self.room_width,
            room_height=self.room_height,
            no_overlap=self.no_overlap,
            map_type=self.map_type,
            indoor_prob=self.indoor_prob,
            seed=0
        )
        make_image(map, self.ns)
        rospy.loginfo("Initial random map generated.")

    def generate_mapdata(self, seed: int = 0):  # generate random map data array for occupancy grid
        map = create_random_map(
            height=self.height,
            width=self.width,
            corridor_radius=self.corridor_radius,
            iterations=self.iterations,
            obstacle_number=self.obstacle_number,
            obstacle_extra_radius=self.obstacle_extra_radius,
            room_number=self.room_number,
            room_width=self.room_width,
            room_height=self.room_height,
            no_overlap=self.no_overlap,
            map_type=self.map_type,
            indoor_prob=self.indoor_prob,
            seed=seed
        )
        make_image(map, self.ns)
        map = np.flip(map, axis=0)
        # map currently [0,1] 2D np array needs to be flattened for publishing OccupancyGrid.data
        map = (map * 100).flatten()
        return map

    # def new_episode_callback(self,goal_msg: PoseStamped):
    #     current_episode = goal_msg.header.seq
    #     is_new_episode = self.nr != current_episode # self.nr starts with -1 so 0 will be the first new episode
    #     if is_new_episode:
    #         self.nr = current_episode
    #         self.occupancy_grid.data = self.generate_mapdata()
    #         rospy.loginfo("New random map generated for episode {}.".format(self.nr))
    #         self.mappub.publish(self.occupancy_grid)
    #         rospy.loginfo("New random map published.")

    def new_episode_callback(self, request: GetMapWithSeedRequest):
        seed = request.seed
        self.occupancy_grid.data = self.generate_mapdata(seed)
        self.mappub.publish(self.occupancy_grid)
        srv_response = GetMapWithSeedResponse(map=self.occupancy_grid)
        return srv_response


if __name__ == '__main__':
    rospy.init_node('map_generator')
    # if rospy.get_param("map_file") == "random_map":
    task_generator = MapGenerator()
    rospy.spin()