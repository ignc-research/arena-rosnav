#!/usr/bin/env python
import numpy as np
import rospy
import sys
import time
from pedsim_msgs.msg import LineObstacles
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

class Publisher:
    def __init__(self):
        rospy.init_node('pedsim_walls_publisher', anonymous=True)
        self.simulated_walls = None
        self.marker_pub = rospy.Publisher(rospy.get_namespace() + 'pedsim_walls', MarkerArray, queue_size=10)
        self.walls_sub = rospy.Subscriber(rospy.get_namespace() + "pedsim_simulator/simulated_walls", LineObstacles, self.simulated_walls_callback)
        self.num_markers = 0
        self.last_walls_callback = rospy.get_rostime()

    def simulated_walls_callback(self, walls_msg):
        self.simulated_walls = walls_msg
        self.last_walls_callback = rospy.get_rostime()
        self.remove_old_markers()

    def remove_all_markers(self):
        removal_marker = Marker()
        removal_marker.header.frame_id = "map"
        removal_marker.action = Marker.DELETEALL
        markers = MarkerArray()
        markers.markers.append(removal_marker)
        self.marker_pub.publish(markers)

    def remove_old_markers(self):
        if self.simulated_walls == None:
            return

        # not receiving an agent_states-callback for some time
        # means that there are no agents
        # -> delete all markers
        if (rospy.get_rostime() - self.last_walls_callback) > rospy.Duration(1.0):
            self.remove_all_markers()
            return

        # delete all markers if number changes
        if self.num_markers != len(self.simulated_walls.obstacles):
            self.num_markers = len(self.simulated_walls.obstacles)
            self.remove_all_markers()
            return

    def create_linestrip_marker(self, id, points, r, g, b, a = 1.0):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "marker"
        marker.id = id
        marker.type = Marker.LINE_STRIP
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
        marker.scale.z = 0.1
        marker.color.a = a
        marker.color.r = r * 255.0
        marker.color.g = g * 255.0
        marker.color.b = b * 255.0
        marker.points = points
        return marker

    def run(self):
        rate = 1.0 / 50.0  # 50Hz
        while not rospy.is_shutdown():
            # wait for walls to be filled
            if self.simulated_walls is None:
                continue
            else:
                markers = MarkerArray()

                # iterate over walls
                for i, wall in enumerate(self.simulated_walls.obstacles):
                    start = Point(wall.start.x, wall.start.y, 0)
                    end = Point(wall.end.x, wall.end.y, 0)
                    marker = self.create_linestrip_marker(i, [start, end], 0, 0, 1)
                    markers.markers.append(marker)

                self.marker_pub.publish(markers)

            time.sleep(rate)

if __name__ == '__main__':
    publisher = Publisher()
    try:
        publisher.run()
    except rospy.ROSInterruptException:
        pass
