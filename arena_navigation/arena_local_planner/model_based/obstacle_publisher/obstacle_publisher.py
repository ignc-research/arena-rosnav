 #!/usr/bin/env python
from visualization_msgs.msg import MarkerArray
import rosnode
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import  Point, Twist
from std_msgs.msg import ColorRGBA
from ford_msgs.msg import Clusters




class odom_msg():

    def __init__(self):
        self.num_poses = 0
        self.sub_pose = rospy.Subscriber('/odom',Odometry,self.cbPose)
        self.pub_obst_odom = rospy.Publisher('/obst_odom',Clusters,queue_size=1)
        # static
        self.sub_pose = rospy.Subscriber('/flatland_server/debug/layer/static',MarkerArray,self.getStaticMap)
        

        self.obstacles = {}
        self.num_obst = 0
        self.cluster = Clusters()
        self.add_obst = True

        self.vel = Twist()
        self.vel.angular.z = 0.1
        self.vel.linear.x = 0.12

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
        # obst_ns = "/obstacle_"

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
                rospy.sleep(0.2)
                


        self.add_obst = False
        # reset cluster
        self.cluster = Clusters()
        # fill cluster with obstacle odom
        for i in self.obstacles:
            tmp_point = Point()
            tmp_point.x = self.obstacles[i].pose.pose.position.x 
            tmp_point.y = self.obstacles[i].pose.pose.position.y 
            
            tmp_vel = self.obstacles[i].twist.twist.linear

            self.cluster.mean_points.append(tmp_point)
            self.cluster.velocities.append(tmp_vel)
            self.cluster.labels.append(self.obstacles[i].header.seq)
       
        self.pub_obst_odom.publish(self.cluster)
        self.add_obst = True
    	
        if self.num_obst != len(self.obstacles):
            self.num_obst = len(self.obstacles)
            print("========================================")
            print("Number of obstacles: "+str(self.num_obst))
            for t in obst_topics:
                print(t)

        
    def getStaticMap(self, msg):
        print(type(msg.markers[0]))

def run():
    rospy.init_node('tb3_obstacles',anonymous=False)
    tb3_obst = odom_msg()
    rospy.spin()


if __name__ == '__main__':
    run()





