#!/usr/bin/env python
from visualization_msgs.msg import MarkerArray
import rosnode
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import  Point, Vector3
from std_msgs.msg import ColorRGBA
from ford_msgs.msg import Clusters

import copy


class sensor():

    def __init__(self):
        # tmgr

        # last updated topic
        self.update_cluster = True

        # get robot pos
        # self.sub_pose = rospy.Subscriber('/odom',Odometry,self.cbPose)
        # pub obst pos

        self.obstacles_dyn = {}
        self.obstacles_static = {}

        self.cluster = Clusters()

        # collect static and dynamic obstacles
        self.obst_topics_dyn = []
        self.obst_topics_static = []
        self.get_obstacle_topics()

        self.pub_obst_odom = rospy.Publisher('/obst_odom',Clusters,queue_size=1)
        self.pub_timer = rospy.Timer(rospy.Duration(0.1),self.pub_odom)

        # static obst

                    
    def update_obstacle_odom(self):
        # subscribe 
        for topic in self.obst_topics_dyn:
            rospy.Subscriber(topic,MarkerArray,self.cb_marker, topic) 
        for topic in self.obst_topics_static:
            rospy.Subscriber(topic,MarkerArray,self.cb_marker, topic) 

    def get_obstacle_topics(self):
        topics = rospy.get_published_topics()
        for t_list in topics:
            for t in t_list:
                if "/debug/model/obstacle_dynamic" in t:
                   self.obst_topics_dyn.append(t)
                elif "/debug/model/obstacle_circle" in t:
                    self.obst_topics_static.append(t)
        self.update_obstacle_odom()
        print("========================================")
        print("dynamic obstacles:", len(self.obst_topics_dyn))
        for topic in self.obst_topics_dyn:
            print(topic)
        print("dynamic obstacles:", len(self.obst_topics_static))
        for topic in self.obst_topics_static:
            print(topic)

                   
    def pub_odom(self,event):
        self.update_cluster = False
        self.fill_cluster()
        self.pub_obst_odom.publish(self.cluster)  
        # reset cluster
        self.cluster = Clusters()  
        self.update_cluster = True

    def fill_cluster(self):
        for topic in self.obstacles_dyn:
            tmp_point = Point()
            tmp_point.x = self.obstacles_dyn[topic][0].x
            tmp_point.y = self.obstacles_dyn[topic][0].y
            tmp_point.z = self.obstacles_dyn[topic][1]

            tmp_vel = self.obstacles_dyn[topic][2]
            
            self.cluster.mean_points.append(tmp_point)
            self.cluster.velocities.append(tmp_vel)
            self.cluster.labels.append(self.obstacles_dyn[topic][4])

        # print("=========================================")
        for topic in self.obst_topics_static:
            tmp_point = Point()
            tmp_point.x = self.obstacles_static[topic][0].x
            tmp_point.y = self.obstacles_static[topic][0].y
            tmp_point.z = self.obstacles_static[topic][1]

            tmp_vel = self.obstacles_static[topic][2]
            
            self.cluster.mean_points.append(tmp_point)
            self.cluster.velocities.append(tmp_vel)
            self.cluster.labels.append(self.obstacles_static[topic][3])
        
        # print(self.cluster)
    def cb_marker(self, msg, topic):

        if self.update_cluster:
            # print(topic)
            v = Vector3()
            m = msg.markers[0]
            pos = m.pose.position
            r = m.scale.x/2
            label = 0
            

            if "dynamic" in topic: 
                if topic in self.obstacles_dyn:
                    old_pos = self.obstacles_dyn[topic][0]
                    old_time = self.obstacles_dyn[topic][3].nsecs
                    curr_time = m.header.stamp.nsecs
                    dt = (curr_time-old_time)*10**-9
                    if dt>0:
                        v.x = (pos.x-old_pos.x)/dt
                        v.y = (pos.y-old_pos.y)/dt
                    # update dyn obst
                    label = list(self.obstacles_dyn).index(topic) + 10
                self.obstacles_dyn[topic] = [pos, r, v, m.header.stamp,label]
            
            else:
                # unsub static obst and update
                
                if topic in self.obstacles_static:
                    label = list(self.obstacles_static).index(topic)
                self.obstacles_static[topic] = [pos, r, v, label]

                # self.sub_static.unregister()
            
          
           


    # def cbPose(self, msg):
        
    #     if self.num_obst != len(self.obstacles_dyn):
    #         self.num_obst = len(self.obstacles_dyn)
    #         print("========================================")
    #         print("Number of obstacles: "+str(self.num_obst))
    #         for t in self.obst_topics:
    #             print(t)



def run():
    rospy.init_node('tb3_obstacles',anonymous=False)
    tb3_obst = sensor()
    rospy.spin()


if __name__ == '__main__':
    run()





