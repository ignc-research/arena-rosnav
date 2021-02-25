#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class Listener:
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        self.goal = np.zeros(2)
        self.obstacle = np.zeros(2)
        self.v_prev = np.zeros(2)
        self.v_now = np.zeros(2)
        self.pos_prev = np.zeros(2)
        self.pos_now = np.zeros(2)
        self.time_prev = rospy.Time.now()
        self.time_now = rospy.Time.now() + rospy.Duration.from_sec(1)
        self.pub_F_goal = rospy.Publisher('force_goal_marker', Marker, queue_size=10)
        self.pub_F_obstacle = rospy.Publisher('force_obstacle_marker', Marker, queue_size=10)
        self.last_odom_callback = rospy.Time.now()

    def create_custom_marker(self, F, id, r, g, b):
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
        start_point = Point(self.pos_now[0], self.pos_now[1], 0)
        end_point_array = self.pos_now + F
        end_point = Point(end_point_array[0], end_point_array[1], 0)
        marker.points = [start_point, end_point]
        return marker

    def vis_forces(self, F_obstacle, F_goal):
        obstacle_marker = self.create_custom_marker(F_obstacle, 1, 239, 41, 41)
        self.pub_F_obstacle.publish(obstacle_marker)
        goal_marker = self.create_custom_marker(F_goal, 2, 204, 41, 204)
        self.pub_F_goal.publish(goal_marker)

    def odom_callback(self, odometry):
        time_passed = rospy.Time.now() - self.last_odom_callback
        time_passed = time_passed.to_sec()
        if time_passed > 1.0:
            # get change in position
            self.pos_prev = self.pos_now
            x = odometry.pose.pose.position.x
            y = odometry.pose.pose.position.y
            self.pos_now = np.array([x, y])

            # get time passed
            self.time_prev = self.time_now
            self.time_now = rospy.Time.now()
            time_step = self.time_now - self.time_prev
            time_step = time_step.to_sec()

            # get change in velocity
            if time_step > 0:
                self.v_prev = self.v_now
                self.v_now = (self.pos_now - self.pos_prev) / time_step

                # line base point
                a_goal = self.pos_now + self.v_prev
                # line direction vector
                u_goal = self.goal - self.pos_now

                # line base point
                a_obstacle = self.pos_now + self.v_now
                # line direction vector
                u_obstacle = self.pos_now - self.obstacle

                # solve equations
                a = np.column_stack((u_obstacle, -1 * u_goal))
                b = a_goal - a_obstacle
                x = np.linalg.solve(a, b)

                intersection = a_goal + x[0] * u_goal

                v_obstacle = (self.pos_now + self.v_now) - intersection
                v_goal = intersection - (self.pos_now + self.v_prev)

                m_vehicle = 1
                F_obstacle = m_vehicle * v_obstacle / time_step  # F = m * a = m * v / t
                F_goal = m_vehicle * v_goal / time_step  # F = m * a = m * v / t


                self.vis_forces(F_obstacle, F_goal)

                print("------")
                print("obstacle: ", F_obstacle)
                print("goal: ", F_goal)
            
            self.last_odom_callback = rospy.Time.now()

    def obstacle_callback(self, point):
        x = point.point.x
        y = point.point.y
        self.obstacle = np.array([x, y])

    def goal_callback(self, point):
        x = point.point.x
        y = point.point.y
        self.goal = np.array([x, y])
        
    def listen(self):
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("forces_obstacle", PointStamped, self.obstacle_callback)
        rospy.Subscriber("forces_goal", PointStamped, self.goal_callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    listener = Listener()
    listener.listen()