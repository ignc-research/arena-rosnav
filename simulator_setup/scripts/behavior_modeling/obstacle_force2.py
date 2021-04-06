#!/usr/bin/env python
import numpy as np
import rospy
import sys
import time
from nav_msgs.msg import OccupancyGrid
from pedsim_msgs.msg import AgentStates
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
# np.set_printoptions(threshold=sys.maxsize)

class Listener:
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        self.force_max_distance = 1
        self.map_array = None
        self.agent_positions = None
        self.marker_publishers = []
        self.obstacle_pos_publishers = []
        self.agent_pos_publishers = []
        self.considered_pos_publishers = []
        for i in range(10):
            publisher_force = rospy.Publisher('obstacle_force_' + str(i), Marker, queue_size=10)
            self.marker_publishers.append(publisher_force)
            publisher_obstacle_pos = rospy.Publisher('obstacle_' + str(i), PointStamped, queue_size=10)
            self.obstacle_pos_publishers.append(publisher_obstacle_pos)
            publisher_agent_pos = rospy.Publisher('agent_' + str(i), PointStamped, queue_size=10)
            self.agent_pos_publishers.append(publisher_agent_pos)
            publisher_considered_pos = [rospy.Publisher('considered_pos_' + str(i) + "_" + str(j), PointStamped, queue_size=10) for j in range(10)]
            self.considered_pos_publishers.append(publisher_considered_pos)
        
        self.once = False


    def map_callback(self, map):
        self.map_array = np.array(map.data).reshape(map.info.height, map.info.width)
        # if not self.once:
        #     for i in range(self.map_array.shape[0]):
        #         for j in range(self.map_array.shape[1]):
        #             char = 1 if self.map_array[i,j] > 0 else 0
        #             print(char, end="")
        #             self.once = True
        #         print("")
        self.map = map


    def simulated_agents_callback(self, agent_states_msg):
        # init agent_positions matrix or resize if number of agents changed
        if self.agent_positions is None or len(agent_states_msg.agent_states) != len(self.agent_positions):
            # shape: N_agents x 2
            self.agent_positions = np.zeros((len(agent_states_msg.agent_states), 2))

        for i, agent_state in enumerate(agent_states_msg.agent_states):
            x = agent_state.pose.position.x
            y = agent_state.pose.position.y
            self.agent_positions[i][0] = x
            self.agent_positions[i][1] = y


    def create_custom_marker(self, start, end, id, r, g, b):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.3
        marker.scale.z = 0.0
        marker.color.a = 1.0
        marker.color.r = r / 255.0
        marker.color.g = g / 255.0
        marker.color.b = b / 255.0
        start_point = Point(start[0], start[1], 0)
        end_point = Point(end[0], end[1], 0)
        marker.points = [start_point, end_point]
        return marker


    # def vis_forces(self, forces):
    #     # only publish as many forces as we have publishers
    #     for force, publisher in zip(forces[:len(self.marker_publishers)], self.marker_publishers):
    #         marker = self.create_custom_marker(force, 1, 239, 41, 41)
    #         publisher.publish(marker)


    def vis_force(self, id, force, origin):
        if id < len(self.marker_publishers):
            marker = self.create_custom_marker(origin, origin + force, 1, 239, 41, 41)
            self.marker_publishers[id].publish(marker)


    def vis_obstacle_pos(self, id, pos):
        if not pos is None:
            if id < len(self.obstacle_pos_publishers):
                point = PointStamped(header=Header(stamp=rospy.Time.now(), frame_id="odom"), point=Point(pos[0], pos[1], 0))
                self.obstacle_pos_publishers[id].publish(point)


    def vis_agent_pos(self, id, pos):
        if id < len(self.agent_pos_publishers):
            point = PointStamped(header=Header(stamp=rospy.Time.now(), frame_id="odom"), point=Point(pos[0], pos[1], 0))
            self.agent_pos_publishers[id].publish(point)


    def vis_considered_pos(self, id, positions):
        if id < len(self.considered_pos_publishers):
            for i, pos in enumerate(positions):
                point = PointStamped(header=Header(stamp=rospy.Time.now(), frame_id="odom"), point=Point(pos[0], pos[1], 0))
                self.considered_pos_publishers[id][i].publish(point)


    def odom_pos_to_map_index(self, pos):
        index_x = (pos[0]  - self.map.info.origin.position.x) / self.map.info.resolution
        index_x = int(index_x)
        index_y = (pos[1] - self.map.info.origin.position.y) / self.map.info.resolution
        index_y = int(index_y)
        # print(f"pos y: {pos[1]} pos x: {pos[0]}, index y: {index_y} index x: {index_x}")
        return index_y, index_x

    def is_occupied(self, pos):
        # checks if there is an occupancy probability > 0 at the given position in the map
        index_y, index_x = self.odom_pos_to_map_index(pos)
        if 0 <= index_y < self.map_array.shape[0] and 0 <= index_x < self.map_array.shape[1]:
            return self.map_array[index_y, index_x] > 0

        # one or more indexes are out of bounds
        return True


    def get_closest_obstacle_pos(self, considered_positions, agent_pos):
        # args:
        # considered_positions: list of 2D numpy arrays
        # agent_pos: 2D numpy array
        closest_obstacle_pos = None
        closest_distance = np.inf
        for pos in considered_positions:
            # check if pos is occupied
            if self.is_occupied(pos):
                distance = np.linalg.norm(agent_pos - pos)
                if distance < closest_distance:
                    closest_distance = distance
                    closest_obstacle_pos = pos
        return closest_obstacle_pos, closest_distance



        #     # check if indexes are in range
        #     if 0 <= pos[0] < self.map_array.shape[0] and 0 <= pos[1] < self.map_array.shape[1]:
        #         # check if cell is occupied
        #         if self.map_array[pos[0], pos[1]] > 0:
        #             distance = np.linalg.norm(agent_pos - pos)
        #             if distance < closest_distance:
        #                 closest_distance = distance
        #                 closest_obstacle_pos = pos
        # return closest_obstacle_pos, closest_distance


    def obstacle_force(self, distance):
        if distance <= 0:
            return 0.0
        return 1 / distance


    def show_obstacle_force(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            # wait for map and agent positions to be filled
            if self.map_array is None or self.agent_positions is None:
                continue
            else:
                # iterate over agents
                # forces = []
                for i in range(self.agent_positions.shape[0]):
                    # TODO hier müsste eine transformation stattfinden, von odom nach ???
                    y = int(self.agent_positions[i][0])
                    x = int(self.agent_positions[i][1])
                    # collect cells around agents position
                    considered_positions = []
                    for j in np.linspace(y - self.force_max_distance, y + self.force_max_distance + 1, num=3):
                        for k in np.linspace(x - self.force_max_distance, x + self.force_max_distance + 1, num=3):
                            considered_positions.append(np.array([j, k]))
                    closest_obstacle_pos, distance = self.get_closest_obstacle_pos(considered_positions, self.agent_positions[i])

                    self.vis_agent_pos(i, self.agent_positions[i])
                    self.vis_considered_pos(i, considered_positions)

                    force = np.zeros(2)
                    if not closest_obstacle_pos is None:
                        force_magnitude = self.obstacle_force(distance)
                        force_direction = self.agent_positions[i] - closest_obstacle_pos
                        force = (force_direction / np.linalg.norm(force_direction)) * force_magnitude

                    if i == 0:
                        self.vis_force(i, force, self.agent_positions[i])
                        self.vis_obstacle_pos(i, closest_obstacle_pos)
                        # print("agent: ", self.agent_positions[0])
                        # print("obstacle: ", closest_obstacle_pos)

            rate.sleep()


    def listen(self):
        rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size = 1)
        rospy.Subscriber("pedsim_simulator/simulated_agents", AgentStates, self.simulated_agents_callback)
        self.show_obstacle_force()


if __name__ == '__main__':
    listener = Listener()
    try:
        listener.listen()
    except rospy.ROSInterruptException:
        pass
