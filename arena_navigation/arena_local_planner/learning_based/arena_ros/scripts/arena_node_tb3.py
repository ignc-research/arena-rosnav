#!/usr/bin/env python
# ros
import rospy
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
# viz
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
# arena 
import fc
import math
from torch.nn.utils.rnn import pack_sequence
import torch
import numpy as np

class NN_tb3():
    def __init__(self):

        # 
        self.distance = 0
        self.desired_action = 0
        self.psi = 0
        self.deg_phi = 0
        self.global_goal = PoseStamped()
        self.goal = PoseStamped()
        self.sub_goal = Vector3()
        self.scan = LaserScan()
        # subs
        self.sub_pose = rospy.Subscriber('/odom',Odometry,self.cbPose)
        self.sub_global_goal = rospy.Subscriber('/goal',PoseStamped, self.cbGlobalGoal)
        self.sub_subgoal = rospy.Subscriber('/subgoal',PoseStamped, self.cbSubGoal)
        # self.sub_subgoal = rospy.Subscriber('/plan_manager/subgoal',PoseStamped, self.cbSubGoal)
        self.sub_scan = rospy.Subscriber('/scan',LaserScan, self.cbScan)
        # pubs
        self.pub_twist = rospy.Publisher('/cmd_vel',Twist,queue_size=1) 

        rospy.sleep(5)

        self.nn_timer = rospy.Timer(rospy.Duration(0.01),self.cbComputeActionArena)
        self.control_timer = rospy.Timer(rospy.Duration(0.1),self.cbControl)


    def cbScan(self,msg):
        self.scan = msg

    def cbGlobalGoal(self,msg):
        self.global_goal = msg
        self.goal.pose.position.x = msg.pose.position.x
        self.goal.pose.position.y = msg.pose.position.y
        self.goal.header = msg.header

        # reset subgoals
        print("new goal: "+str([self.goal.pose.position.x,self.goal.pose.position.y])) 

    def cbSubGoal(self,msg):
        self.sub_goal.x = msg.pose.position.x
        self.sub_goal.y = msg.pose.position.y


    def cbPose(self, msg):
        # calculate angle
        q = msg.pose.pose.orientation
        self.psi = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)) # bounded by [-pi, pi]
        v_p = msg.pose.pose.position
        v_g = self.sub_goal
        v_pg = np.array([v_g.x-v_p.x,v_g.y-v_p.y])
        v_pose = np.array([math.cos(self.psi),math.sin(self.psi)])
        angle = np.math.atan2(np.linalg.det([v_pose,v_pg]),np.dot(v_pose,v_pg))
        self.distance = np.linalg.norm(v_pg)
        self.deg_phi = math.degrees(angle)
        # quaternion
       
        # pose
        self.pose = msg.pose
  

        # self.visualize_pose(msg.pose.pose.position,msg.pose.pose.orientation)

    def goalReached(self):
        # how far ?
        if self.distance > 0.3:
            return False
        else:
            return True

    def stop_moving(self):
        twist = Twist()
        # print(twist)
        self.pub_twist.publish(twist)

    def update_action(self, action):
        # print 'update action'
        self.desired_action = action

    def cbControl(self,event):
        self.performAction(self.desired_action)
        return
            
    def cbComputeActionArena(self,event):
        if not self.goalReached():
            NUM_ACTIONS = 5
            num_observations=362
            SEQ_LENGTH=64
            SEQ_LENGTH_MAX=300
            # input
            # pack goal position relative to robot
            angle = self.deg_phi    
            distance = self.distance   
            #lidarscan
            sample = np.asanyarray(self.scan.ranges)
            sample[np.isnan(sample)] = 3.5
            sample=sample.tolist()
            # print(len(sample))

            # print("===============================")
            
            # sample=np.ones([360,]).tolist()
            # print(sample)

            observation=[distance]+[angle]+sample
            #load NN
            model_name="dqn_agent_best_fc_l2.dat"
            net = fc.FC_DQN(num_observations, NUM_ACTIONS)
            net.train(False)# set training mode to false to deactivate dropout layer

            net.load_state_dict(torch.load(model_name, map_location=torch.device('cpu')));

            ##output NN
            # passing observation through net
            state_v = torch.FloatTensor([observation]).to('cpu')
            q_vals_v = net(state_v)
            # select action with max q value
            _, act_v = torch.max(q_vals_v, dim=1)
            action = int(act_v.item())
            self.update_action(action)

        else:
            # print(self.global_goal.pose.position)
            # print("stop moving: "+str(self.distance))
            self.stop_moving()
            return

    def performAction(self, action):

        action_space = {0: [0.2,0],1: [0.15,0.75],2: [0.15,-0.75],3: [0.0,1.5],4: [0.0,-1.5]}
        # action_space = {0: [0.2,0], 1: [0.15,0.35], 2: [0.15,-0.35], 3: [0.0,0.75], 4: [0.0,-0.75]}
        # print(action)
        twist = Twist()
        twist.linear.x = action_space[action][0]
        twist.angular.z = action_space[action][1]

        print("action "+str(action)+": "+str(action_space[action]))
        print("twist: "+str([twist.linear.x, twist.angular.z]))
        # print((sample))
        self.pub_twist.publish(twist)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down Node.")
        self.stop_moving()
        # rospy.loginfo("Stopped %s's velocity." %(self.veh_name))

def run():

    rospy.init_node('arena_tb3',anonymous=False)
    print('==================================\narena node started\n==================================')

    nn_tb3 = NN_tb3()
    rospy.on_shutdown(nn_tb3.on_shutdown)

    rospy.spin()

if __name__ == '__main__':
    run()