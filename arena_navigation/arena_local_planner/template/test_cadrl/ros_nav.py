import rospy
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from std_msgs.msg import ColorRGBA
import copy

class RosNav():
    def __init__(self, goal_topic, subgoal_topic):
        # start node
        # rospy.init_node("robot_state", anonymous=False)
        # print('==================================\nrobot_state-node started\n==================================')
        # rospy.spin()

        # # 
        # position
        self.pose = PoseStamped()
        self.sub_goal =  Vector3()
        self.global_goal =  Vector3()
        self.vel = Vector3()
        self.angle_pose = 0.0
        self.goal_tol = 0.3
        self.radius = 0.3
        self.distance_gg = 0
        # obstacles 
        self.obstacles_state = []
        # viz
        self.num_poses = 0
        # # subscriber
        self.sub_pose = rospy.Subscriber('/odom',Odometry,self.cbPose)
        self.sub_global_goal = rospy.Subscriber(goal_topic,PoseStamped, self.cbGlobalGoal)
        self.sub_subgoal = rospy.Subscriber(subgoal_topic,PoseStamped, self.cbSubGoal)
        # # publisher
        self.pub_twist = rospy.Publisher('/cmd_vel',Twist,queue_size=1) 
        self.pub_path_marker = rospy.Publisher('/visualizer/path',Marker,queue_size=1)



    
    def cbVel(self, msg):
        self.vel = msg.twist.twist.linear

    def cbPose(self,msg):
        self.num_poses += 1
        self.cbVel(msg)
        self.pose = msg.pose
        # pose angle 
        q = msg.pose.pose.orientation
        self.angle_pose = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        # vectors
        v_p = msg.pose.pose.position
        v_g = self.global_goal
        v_pg = np.array([v_g.x-v_p.x,v_g.y-v_p.y])
        self.distance_gg = np.linalg.norm(v_pg)


        self.visualize_path()

    def cbGlobalGoal(self,msg):
        self.global_goal.x = msg.pose.position.x
        self.global_goal.y = msg.pose.position.y

    def cbSubGoal(self,msg):
        self.sub_goal.x = msg.pose.position.x
        self.sub_goal.y = msg.pose.position.y

    def goalReached(self):
        # how far ?
        if self.distance_gg > self.goal_tol:
            return False
        else:
            return True

    def stop_moving(self):
        twist = Twist()
        # print(twist)
        self.pub_twist.publish(twist)
    
    
    # marker
    def visualize_path(self):
            # pose arrow
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'map'
            marker.ns = 'pose'
            marker.id = 0
            marker.type = marker.ARROW
            marker.action = marker.ADD
            marker.pose.position = self.pose.pose.position
            marker.pose.orientation = self.pose.pose.orientation
            marker.scale = Vector3(x=1,y=0.15,z=0.0)
            marker.color = ColorRGBA(b=0.0,g=0,r=0,a=1)
            marker.lifetime = rospy.Duration(60)
            self.pub_path_marker.publish(marker)

            # trail
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'map'
            marker.ns = 'path_trail'
            marker.id = self.num_poses 
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.pose.position = copy.deepcopy(self.pose.pose.position)
            marker.scale = Vector3(x=0.2,y=0.2,z=0.2)
            marker.color = ColorRGBA(g=0.0,r=0,b=1.0,a=0.3)
            marker.lifetime = rospy.Duration(60)
            self.pub_path_marker.publish(marker)