import rospy
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from std_msgs.msg import ColorRGBA
import copy
from ford_msgs.msg import Clusters

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
        self.goal_tol = 0.3
        self.radius = 0.3
        self.distance_gg = 0
        # angles
        self.angle_pose = 0.0
        self.angle_2goal = 0.0
        # action
        self.raw_action = np.zeros((2,))
        # obstacles 
        self.obstacles_state = {}
        # viz
        self.num_poses = 0
        # # subscriber
        self.sub_pose = rospy.Subscriber('/odom',Odometry,self.cbPose)
        self.sub_global_goal = rospy.Subscriber(goal_topic,PoseStamped, self.cbGlobalGoal)
        self.sub_subgoal = rospy.Subscriber(subgoal_topic,PoseStamped, self.cbSubGoal)
        # # publisher
        self.pub_twist = rospy.Publisher('/cmd_vel',Twist,queue_size=1) 
        self.pub_path_marker = rospy.Publisher('/visualizer/path',Marker,queue_size=1)
        # self.pub_agent_markers = rospy.Publisher('/other_agents_markers',MarkerArray,queue_size=1)


        self.use_clusters = False
        # self.use_clusters = False
        if self.use_clusters:
            self.sub_clusters = rospy.Subscriber('/obst_odom',Clusters, self.cbClusters)

    def cbClusters(self,msg):
            # print(msg)
            


            xs = []; ys = []; radii = []; labels = []; heading_angles = []
            num_clusters = len(msg.mean_points)
            # print(num_clusters)
            for i in range(num_clusters):
                index = msg.labels[i]
                x = msg.mean_points[i].x; y = msg.mean_points[i].y
                v_x = msg.velocities[i].x; v_y = msg.velocities[i].y
                # radius = PED_RADIUS
                # lower_r = np.linalg.norm(np.array([msg.mean_points[i].x-msg.min_points[i].x, msg.mean_points[i].y-msg.min_points[i].y]))
                # upper_r = np.linalg.norm(np.array([msg.mean_points[i].x-msg.max_points[i].x, msg.mean_points[i].y-msg.max_points[i].y]))
                inflation_factor = 1.5
                # radius = max(PED_RADIUS, inflation_factor * max(upper_r, lower_r))
                
                radius = msg.mean_points[i].z*inflation_factor

                xs.append(x); ys.append(y); radii.append(radius); labels.append(index); 
                # helper fields
                heading_angle = np.arctan2(v_y, v_x)
                heading_angles = heading_angle
                

            self.obstacles_state["pos"] = msg.mean_points
            self.obstacles_state["v"] = msg.velocities
            self.obstacles_state["label"] = labels

            self.visualize_other_agents(xs, ys, radii, labels)
            # print(self.obstacles_state["v"])
            print(v_x)
          

    def update_action(self, action):
        # print 'update action'
        self.raw_action = action

    
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

    def visualize_other_agents(self,xs,ys,radii,labels):
        markers = MarkerArray()
        for i in range(len(xs)):
            # Orange box for other agent
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'map'
            marker.ns = 'other_agent'
            marker.id = labels[i]
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.pose.position.x = xs[i]
            marker.pose.position.y = ys[i]
            # marker.pose.orientation = orientation
            marker.scale = Vector3(x=2*radii[i],y=2*radii[i],z=1)
            if labels[i] <= 23: # for static map
                # print sm
                marker.color = ColorRGBA(r=0.5,g=0.4,a=1.0)
            else:
                marker.color = ColorRGBA(r=1.0,g=0.4,a=1.0)
            marker.lifetime = rospy.Duration(0.5)
            markers.markers.append(marker)

        self.pub_agent_markers.publish(markers)