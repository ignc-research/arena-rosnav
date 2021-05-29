#!/usr/bin/env python
import numpy as np
import rospy
import sys
import time
from pedsim_msgs.msg import Waypoint
from pedsim_msgs.msg import Waypoints
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

class Publisher:
    def __init__(self):
        rospy.init_node('waypoints_publisher', anonymous=True)
        self.simulated_waypoints = None
        self.marker_pub = rospy.Publisher(rospy.get_namespace() + 'pedsim_waypoint_markers', MarkerArray, queue_size=10)
        self.waypoints_sub = rospy.Subscriber(rospy.get_namespace() + "pedsim_simulator/simulated_waypoints", Waypoints, self.simulated_waypoints_callback)
        self.num_markers = 0
        self.last_waypoints_callback = rospy.get_rostime()

    def simulated_waypoints_callback(self, msg):
        self.simulated_waypoints = msg
        self.last_waypoints_callback = rospy.get_rostime()
        self.remove_old_markers()

    def remove_all_markers(self):
        removal_marker = Marker()
        removal_marker.header.frame_id = "map"
        removal_marker.action = Marker.DELETEALL
        markers = MarkerArray()
        markers.markers.append(removal_marker)
        self.marker_pub.publish(markers)

    def remove_old_markers(self):
        if self.simulated_waypoints == None:
            return

        # not receiving an waypoints-callback for some time
        # means that there are no waypoints
        # -> delete all markers
        if (rospy.get_rostime() - self.last_waypoints_callback) > rospy.Duration(1.0):
            self.remove_all_markers()
            return

        # delete all markers if number changes
        if self.num_markers != len(self.simulated_waypoints.waypoints):
            self.num_markers = len(self.simulated_waypoints.waypoints)
            self.remove_all_markers()
            return

    def create_circle_marker(self, id, x, y, radius, r = 0.0, g = 0.0, b = 1.0, a = 0.2):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "marker"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.MODIFY
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2 * radius
        marker.scale.y = 2 * radius
        marker.scale.z = 1.0
        marker.color.a = a
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.points = []
        marker.text = ""
        return marker

    def publish_waypoints(self):
        rate = 1.0 / 50.0  # 50Hz
        while not rospy.is_shutdown():
            # wait for waypoints to be filled
            if self.simulated_waypoints is None:
                continue
            else:
                markers = MarkerArray()

                # iterate over waypoints
                for i, wp in enumerate(self.simulated_waypoints.waypoints):
                    id = i
                    x = wp.position.x
                    y = wp.position.y
                    r = wp.radius
                    # take interaction radius if waypoint is interactive
                    if wp.type == 2:
                        r = wp.interaction_radius
                    marker = self.create_circle_marker(id, x, y, r)
                    markers.markers.append(marker)

                self.marker_pub.publish(markers)
                self.num_markers = len(markers.markers)

            time.sleep(rate)

    def run(self):
        self.publish_waypoints()


if __name__ == '__main__':
    publisher = Publisher()
    try:
        publisher.run()
    except rospy.ROSInterruptException:
        pass
