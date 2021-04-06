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

class Listener:
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        self.simulated_agents = None
        self.marker_publishers = []
        for i in range(10):
            publisher_force = rospy.Publisher('obstacle_force_' + str(i), Marker, queue_size=10)
            self.marker_publishers.append(publisher_force)
        

    def simulated_agents_callback(self, agent_states_msg):
        self.simulated_agents = agent_states_msg


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


    def vis_force(self, id, force, origin):
        if id < len(self.marker_publishers):
            marker = self.create_custom_marker(origin, origin + force, 1, 239, 41, 41)
            self.marker_publishers[id].publish(marker)


    def show_obstacle_force(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            # wait for map and agent positions to be filled
            if self.simulated_agents is None:
                continue
            else:
                # iterate over agents
                for agent in self.simulated_agents.agent_states:
                    id = agent.id
                    pos_x = agent.pose.position.x
                    pos_y = agent.pose.position.y
                    obstacle_force_x = agent.forces.obstacle_force.x
                    obstacle_force_y = agent.forces.obstacle_force.y

                    self.vis_force(id, np.array([obstacle_force_x, obstacle_force_y]), np.array([pos_x, pos_y]))

            rate.sleep()

    def listen(self):
        rospy.Subscriber("pedsim_simulator/simulated_agents", AgentStates, self.simulated_agents_callback)
        self.show_obstacle_force()


if __name__ == '__main__':
    listener = Listener()
    try:
        listener.listen()
    except rospy.ROSInterruptException:
        pass
