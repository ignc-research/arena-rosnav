#!/usr/bin/env python3
from visualization_msgs.msg import MarkerArray
import rosnode
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import  Point, Twist
from std_msgs.msg import ColorRGBA
from ford_msgs.msg import Clusters

import copy


class sensor():

    def __init__(self):
        self.num_poses = 0
        self.sub_pose = rospy.Subscriber('/odom',Odometry,self.cbPose)
        self.pub_obst_odom = rospy.Publisher('/obst_odom',Clusters,queue_size=1)

        self.obstacles = {}
        self.num_obst = 0
        self.cluster = Clusters()
        self.add_obst = True
        # obst vel
        self.vel = Twist()
        self.vel.angular.z = rospy.get_param("~vz")
        self.vel.linear.x = rospy.get_param("~vx")
        self.radius = 0.5
        # static map
        self.static_map_obst = Clusters()


        self.pub_timer = rospy.Timer(rospy.Duration(0.1),self.pubOdom)

    def pubOdom(self,event):
        self.pub_obst_odom.publish(self.cluster)
        self.add_obst = True
        self.cluster = copy.deepcopy(self.static_map_obst)



    def appendMapObst(self,x,y,r):
            tmp_point = Point()
            tmp_point.x = x
            tmp_point.y = y
            tmp_point.z = r
            
            tmp_vel = Twist().linear

            self.static_map_obst.mean_points.append(tmp_point)
            self.static_map_obst.velocities.append(tmp_vel)
            self.static_map_obst.labels.append(len(self.static_map_obst.labels)+1)
            self.static_map_obst.counts.append(1)



    def pubVel(self, topic, twist):
        pub_vel = rospy.Publisher(topic,Twist,queue_size=1)
        pub_vel.publish(twist)

    def cbLog(self, msg, topic):
        # get obstacle odom by name (topic)
        self.obstacles[topic] = msg


    def cbPose(self, msg):


        # get all topics
        topics = rospy.get_published_topics()
        obst_ns = "myrobot_model" 

        obst_topics = []
        # filter topics with ns (obst)
        for t_list in topics:
            for t in t_list:
                if obst_ns in t and "ground_truth" in t:
                   obst_topics.append(t)
        # add obst topic to dictionary
        if self.add_obst:
            for topic in obst_topics:
                rospy.Subscriber(topic,Odometry,self.cbLog, topic)
                v_topic = topic.replace("odometry/ground_truth", "cmd_vel")
                # publish velocity to move obstacles
                self.pubVel(v_topic,self.vel)
                # rospy.sleep(0.2)
                


        self.add_obst = False

        
        # fill cluster with obstacle odom
        for i in self.obstacles:
            tmp_point = Point()
            tmp_point.x = self.obstacles[i].pose.pose.position.x 
            tmp_point.y = self.obstacles[i].pose.pose.position.y 
            tmp_point.z = self.radius

            tmp_vel = self.obstacles[i].twist.twist.linear
            
            # print type(tmp_vel.z)

            self.cluster.mean_points.append(tmp_point)
            self.cluster.velocities.append(tmp_vel)
            self.cluster.labels.append(self.obstacles[i].header.seq)
            self.cluster.counts.append(0)

       
        # print(len(self.cluster.mean_points))

    	
        if self.num_obst != len(self.obstacles):
            self.num_obst = len(self.obstacles)
            print("========================================")
            print("Number of obstacles: "+str(self.num_obst))
            for t in obst_topics:
                print(t)



def run():
    rospy.init_node('tb3_obstacles',anonymous=False)
    tb3_obst = sensor()
    # create static map 
    static_map = rospy.get_param("~usm")
    if static_map:
        
        # 1
        tb3_obst.appendMapObst(24.5,15.53,1.2)
        tb3_obst.appendMapObst(24.3,17.7,1.2)
        # 2
        tb3_obst.appendMapObst(19.8,11.63,0.7)
        tb3_obst.appendMapObst(18.2,11.63,0.7)
        tb3_obst.appendMapObst(16.5,11.63,0.7)
        tb3_obst.appendMapObst(14.8,11.63,0.7)
        # 3
        tb3_obst.appendMapObst(20,2.6,1.3)
        tb3_obst.appendMapObst(18.4,1.34,1)
        tb3_obst.appendMapObst(19,-0.19,1)
        # 4
        tb3_obst.appendMapObst(16.5,18.5,0.7)
        tb3_obst.appendMapObst(14.4,18,0.8)
        tb3_obst.appendMapObst(12,17.7,1.2)
        # 5
        tb3_obst.appendMapObst(14,6,1.7)
        # 6
        tb3_obst.appendMapObst(12.2,-1.2,1.3)
        tb3_obst.appendMapObst(9.22,-1.2,1.3)
        # 7
        tb3_obst.appendMapObst(0.74,14.3,1.7)
        tb3_obst.appendMapObst(4.4,14.3,1.7)
        # 8
        tb3_obst.appendMapObst(3.6,6.4,1.3)
        tb3_obst.appendMapObst(3.6,3.6,1.3)
        # 9
        tb3_obst.appendMapObst(0,-2.67,1)
        # 10
        tb3_obst.appendMapObst(-4.2,9.36,1)
        tb3_obst.appendMapObst(-3.8,7.2,1)
        tb3_obst.appendMapObst(-3.84,5.17,1)
        tb3_obst.appendMapObst(-4.23,3.277,1)


    rospy.spin()


if __name__ == '__main__':
    run()





