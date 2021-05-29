#!/usr/bin/env python
import numpy as np
import rospy
import sys
import time
from pedsim_msgs.msg import AgentStates
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

class Publisher:
    def __init__(self):
        rospy.init_node('pedsim_forces_publisher', anonymous=True)
        self.simulated_agents = None
        self.marker_pub = rospy.Publisher(rospy.get_namespace() + 'pedsim_forces', MarkerArray, queue_size=10)
        self.agent_states_sub = rospy.Subscriber(rospy.get_namespace() + "pedsim_simulator/simulated_agents", AgentStates, self.simulated_agents_callback)
        self.num_markers = 0
        self.last_agent_states_callback = rospy.get_rostime()

    def simulated_agents_callback(self, agent_states_msg):
        self.simulated_agents = agent_states_msg
        self.last_agent_states_callback = rospy.get_rostime()
        self.remove_old_markers()

    def remove_all_markers(self):
        removal_marker = Marker()
        removal_marker.header.frame_id = "map"
        removal_marker.action = Marker.DELETEALL
        markers = MarkerArray()
        markers.markers.append(removal_marker)
        self.marker_pub.publish(markers)

    def remove_old_markers(self):
        if self.simulated_agents == None:
            return

        # not receiving an agent_states-callback for some time
        # means that there are no agents
        # -> delete all markers
        if (rospy.get_rostime() - self.last_agent_states_callback) > rospy.Duration(1.0):
            self.remove_all_markers()
            return

        # delete all markers if number changes
        if self.num_markers != len(self.simulated_agents.agent_states):
            self.num_markers = len(self.simulated_agents.agent_states)
            self.remove_all_markers()
            return

    def create_arrow_marker(self, id, start_point, end_point, r, g, b, a = 1.0, marker_type = Marker.ARROW):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "marker"
        marker.id = id
        marker.type = marker_type
        marker.action = Marker.MODIFY
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
        marker.color.a = a
        marker.color.r = r * 255.0
        marker.color.g = g * 255.0
        marker.color.b = b * 255.0
        marker.points = [start_point, end_point]
        return marker

    def run(self):
        rate = 1.0 / 50.0  # 50Hz
        while not rospy.is_shutdown():
            # wait for agent positions to be filled
            if self.simulated_agents is None:
                continue
            else:
                markers = MarkerArray()

                # iterate over agents
                for agent in self.simulated_agents.agent_states:
                    id = agent.id * 1000
                    pos_x = agent.pose.position.x
                    pos_y = agent.pose.position.y
                    start_point = Point(pos_x, pos_y, 0.0)
                    # desired force
                    desired_force = agent.forces.desired_force
                    # print(desired_force)
                    end_point = Point(pos_x + desired_force.x, pos_y + desired_force.y, 0.0)
                    marker = self.create_arrow_marker(id+1, start_point, end_point, 0.0, 1.0, 0.0) # green
                    markers.markers.append(marker)
                    # obstacle force
                    obstacle_force = agent.forces.obstacle_force
                    end_point = Point(pos_x + obstacle_force.x, pos_y + obstacle_force.y, 0.0)
                    marker = self.create_arrow_marker(id+2, start_point, end_point, 1.0, 0.0, 0.0) # red
                    markers.markers.append(marker)
                    # social force
                    social_force = agent.forces.social_force
                    end_point = Point(pos_x + social_force.x, pos_y + social_force.y, 0.0)
                    marker = self.create_arrow_marker(id+3, start_point, end_point, 0.0, 0.0, 1.0) # blue
                    markers.markers.append(marker)
                    # destination
                    destination = agent.destination
                    end_point = Point(destination.x, destination.y, 0.0)
                    marker = self.create_arrow_marker(id+4, start_point, end_point, 0.0, 0.0, 1.0, 0.3, Marker.LINE_STRIP) # blue
                    markers.markers.append(marker)

                self.marker_pub.publish(markers)

            time.sleep(rate)

if __name__ == '__main__':
    publisher = Publisher()
    try:
        publisher.run()
    except rospy.ROSInterruptException:
        pass
