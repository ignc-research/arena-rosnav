#!/usr/bin/env python
import numpy as np
import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path, Odometry
from ford_msgs.msg import Clusters
# 
class police():
    def __init__(self):
        self.n_col = 0
        self.n_replan_pm = 0
        self.n_replan_mb = 0
        self.collision_flag = False

        self.odom = Odometry()
        self.cluster = Clusters()

        self.update_cluster = True

 
        # sub
        self.scan = rospy.Subscriber('/scan',LaserScan, self.cbScan)
        # rospy.Subscriber('/planning_vis/goal',Marker, self.get_pm_path)
        # rospy.Subscriber('/move_base/DWAPlannerROS/global_plan',Path, self.get_mb_path)
        # rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan',Path, self.get_mb_path)
        rospy.Subscriber('/odom',Odometry, self.cb_odom)
        # rospy.Subscriber('/obst_odom',Clusters, self.cb_cluster)


        # pub
        self.pub_col = rospy.Publisher('police/collision', Int16, queue_size=10)
        # self.pub_mb_replan = rospy.Publisher('police/mb_replanned', Int16, queue_size=10)
        # self.pub_pb_replan = rospy.Publisher('police/pm_replanned', Int16, queue_size=10)
        self.pub_odom = rospy.Publisher('police/odom', Odometry, queue_size=10)
        # self.pub_obst_odom = rospy.Publisher('police/obst_odom',Clusters,queue_size=1)


        rospy.Timer(rospy.Duration(0.5),self.publish_state)


    def cb_cluster(self,msg):
        if self.update_cluster:
            self.cluster = Clusters()
            num_clusters = len(msg.mean_points)
            # print(num_clusters)
            for i in range(num_clusters):
                if num_clusters < 24:
                    self.cluster.mean_points.append(msg.mean_points[i])
                    self.cluster.velocities.append(msg.velocities[i])
                    self.cluster.labels.append(msg.labels[i])
                elif msg.labels[i] >= 24:
                    self.cluster.mean_points.append(msg.mean_points[i])
                    self.cluster.velocities.append(msg.velocities[i])
                    self.cluster.labels.append(msg.labels[i])

        #self.cluster = msg

    def cb_odom(self,msg):
        self.odom = msg

    def get_pm_path(self,msg):
        self.n_replan_pm += 1

    def get_mb_path(self,msg):
        self.n_replan_mb += 1


    def publish_state(self, event):
        # print(self.odom)
        # self.update_cluster = False
        # self.pub_obst_odom.publish(self.cluster) 
        # self.update_cluster = True
        
        self.pub_odom.publish(self.odom)
        # self.pub_mb_replan.publish(self.n_replan_mb)
        # self.pub_pb_replan.publish(self.n_replan_pm)

    def cbScan(self,msg):
        scan_array = np.asarray(msg.ranges)
        d_min = np.nanmin(scan_array)
        if np.isnan(d_min):
            d_min = 3.5

        if d_min > 0.5:
            self.collision_flag = False
        if d_min <= 0.35 and not self.collision_flag:
            self.collision_flag = True
            self.n_col += 1 
            self.pub_col.publish(self.n_col)
            print(self.n_col)
            



def run():
    rospy.init_node('scenario_police',anonymous=False)
    print("watching scene")
    police()
    rospy.spin()


if __name__=="__main__":
    run()

