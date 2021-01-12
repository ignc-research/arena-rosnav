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
import math
import gru
from torch.nn.utils.rnn import pack_sequence
import torch
import numpy as np

class NN_tb3():
    def __init__(self):

        #parameters
        self.episode_idx=0
        self.last_action=-1
        self.last_value=0
        self.distance = 0

        #
        self.psi = 0
        self.phi = 0
        self.global_goal = PoseStamped()
        self.goal = PoseStamped()
        self.sub_goal = Vector3()
        self.scan = LaserScan()
        # subs
        self.sub_pose = rospy.Subscriber('/odom',Odometry,self.cbPose)
        self.sub_global_goal = rospy.Subscriber('/goal',PoseStamped, self.cbGlobalGoal)
        self.sub_subgoal = rospy.Subscriber('/plan_manager/subgoal',PoseStamped, self.cbSubGoal)
        self.sub_subgoal = rospy.Subscriber('/scan',LaserScan, self.cbScan)
        # pubs
        self.pub_pose_marker = rospy.Publisher('/arena_pose',Marker,queue_size=1)
        self.pub_twist = rospy.Publisher('/cmd_vel',Twist,queue_size=1) 

        self.nn_timer = rospy.Timer(rospy.Duration(0.1),self.calculateAction)


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
        # print "new subgoal: "+str(self.sub_goal)

    def cbPlannerMode(self, msg):
        self.operation_mode = msg
        self.operation_mode.mode = self.operation_mode.NN

    def cbPose(self, msg):
        q = msg.pose.pose.orientation
        # p = msg.pose.pose.position
        g = self.global_goal.pose.orientation
        self.phi = np.arctan2(2.0*(g.w*g.z + g.x*g.y), 1-2*(g.y*g.y+g.z*g.z))
        # p = self.global_goal
        self.psi = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)) # bounded by [-pi, pi]
        # self.phi = np.arctan2(2.0*(p.w*p.z + p.x*p.y), 1-2*(p.y*p.y+p.z*p.z))

        self.pose = msg.pose
        # self.visualize_pose(msg.pose.pose.position,msg.pose.pose.orientation)

    def goalReached(self):
        curr_pos = self.pose.pose.position
        goal = self.global_goal.pose.position
        # check dist to goal
        dist = np.array([curr_pos.x-goal.x,curr_pos.y-curr_pos.y])
        self.distance = np.linalg.norm(dist)

        # how far ?
        if self.distance > 0.3:
            return False
        else:
            return True

    def stop_moving(self):
        twist = Twist()
        # print(twist)
        self.pub_twist.publish(twist)

    def cbComputeActionArena(self, event):         
        if not self.goalReached():   
            # Input
            # pack goal position relative to robot
            angle = self.psi     #in rad
            distance = 0    #in meter

            #  laser samples
            sample2 = np.asanyarray(self.scan.ranges)
            sample2[np.isnan(sample2)] = 0
            sample2=sample2.tolist()
            print(sample2)
            # self.goalReached()

            sample=np.zeros([72,]).tolist()
            obervation_input=[angle]+[distance]+sample


            #load NN
            model_name="dqn_agent_best_72_gru.dat"
            net = gru.GRUModel(self.num_observations, self.NUM_ACTIONS)
            net.train(False)# set training mode to false to deactivate dropout layer

            h = net.init_hidden(1).data
            net.load_state_dict(torch.load(model_name, map_location=torch.device('cpu')));

            ##output NN
            prediction=self.step(obervation_input, h, net)
            print(prediction)
        else:
            print("stop moving")
            self.stop_moving()

    def calculateAction(self,event):
        if not self.goalReached():
            #parameters
            NUM_ACTIONS = 5
            num_observations=364
            BATCH_SIZE = 64
            SEQ_LENGTH=64
            SEQ_LENGTH_MAX=300
            tensor_state_buffer = torch.zeros(SEQ_LENGTH_MAX, num_observations,dtype=torch.float)
                
            # Input
            # pack goal position relative to robot
            angle = self.psi-self.phi      #in rad
            angle = math.degrees(angle)
            #lidarscan (laser)
            sample = np.asanyarray(self.scan.ranges)
            sample[np.isnan(sample)] = 0
            sample=sample.tolist()
            # sample=np.zeros([360,]).tolist()
            # print(angle)
            # print(self.distance)
            obervation_input=[self.last_value]+[self.last_action]+[angle]+[self.distance]+sample

            #load NN
            model_name="dqn_agent_best_360_gru.dat"
            net = gru.GRUModel(num_observations, NUM_ACTIONS)
            net.train(False)# set training mode to false to deactivate dropout layer

            h = net.init_hidden(1).data
            net.load_state_dict(torch.load(model_name, map_location=torch.device('cpu')))

            ##output NN
            q = None
            tensor_state_buffer[self.episode_idx] = torch.FloatTensor(obervation_input);
            if self.episode_idx > SEQ_LENGTH-1:
                start_index= self.episode_idx-(SEQ_LENGTH-1)
                L=SEQ_LENGTH
            else:
                start_index = 0
                L=self.episode_idx+1
            state_v=[torch.narrow(tensor_state_buffer, dim=0, start=start_index, length=L)]
            t=pack_sequence(state_v, enforce_sorted=False)
            q,_ = net(t,h)
            q=q.view(-1,NUM_ACTIONS)
            
            # select action with max q value
            _, act_v = torch.max(q, dim=1)
            action = int(act_v.item())
            # print(q)
            self.last_value = q[0][action]
            self.last_action = action
            self.performAction(action)
        else:
            print("stop moving")
            self.stop_moving()

    def performAction(self, action):
        actionLib = {0: [0.2,0],1: [0.15,0.75],2: [0.15,-0.75],3: [0.0,1.5],4: [0.0,-1.5]}
        # print(action)
        twist = Twist()
        # print(actionLib[action][0])
        twist.linear.x = actionLib[action][0]
        twist.angular.z = actionLib[action][1]
   
        # print(twist)
        self.pub_twist.publish(twist)


    def update_subgoal(self,subgoal):
        self.goal.pose.position.x = subgoal[0]
        self.goal.pose.position.y = subgoal[1]

    def visualize_subgoal(self,subgoal, subgoal_options=None):
        markers = MarkerArray()

        # Display GREEN DOT at NN subgoal
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.ns = 'subgoal'
        marker.id = 0
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.pose.position.x = subgoal[0]
        marker.pose.position.y = subgoal[1]
        marker.scale = Vector3(x=0.2,y=0.2,z=0)
        marker.color = ColorRGBA(r=0.0,g=0.0,b=0.0,a=1.0)
        marker.lifetime = rospy.Duration(2.0)
        self.pub_goal_path_marker.publish(marker)

        if subgoal_options is not None:
            for i in xrange(len(subgoal_options)):
                marker = Marker()
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = 'map'
                marker.ns = 'subgoal'
                marker.id = i+1
                # marker.type = marker.CUBE
                marker.type = marker.CYLINDER
                marker.action = marker.ADD
                marker.pose.position.x = subgoal_options[i][0]
                marker.pose.position.y = subgoal_options[i][1]
                marker.scale = Vector3(x=0.2,y=0.2,z=0.2)
                marker.color = ColorRGBA(r=0.0,g=0.0,b=255,a=1.0)
                marker.lifetime = rospy.Duration(1.0)
                self.pub_goal_path_marker.publish(marker)

    def visualize_pose(self,pos,orientation):
        # Yellow Box for Vehicle
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.ns = 'agent'
        marker.id = 0
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position = pos
        marker.pose.orientation = orientation
        marker.scale = Vector3(x=0.7,y=0.42,z=1)
        marker.color = ColorRGBA(r=1.0,g=1.0,a=1.0)
        marker.lifetime = rospy.Duration(1.0)
        self.pub_pose_marker.publish(marker)

        # Red track for trajectory over time
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.ns = 'agent'
        marker.id = self.num_poses
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position = pos
        marker.pose.orientation = orientation
        marker.scale = Vector3(x=0.2,y=0.2,z=0.2)
        marker.color = ColorRGBA(r=1.0,a=1.0)
        marker.lifetime = rospy.Duration(10.0)
        self.pub_pose_marker.publish(marker)

        # print marker

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))
        self.stop_moving()
        rospy.loginfo("Stopped %s's velocity." %(self.veh_name))

def run():

    rospy.init_node('arena_tb3',anonymous=False)

    print('==================================\narena node started\n==================================')

    nn_tb3 = NN_tb3()
    rospy.on_shutdown(nn_tb3.on_shutdown)

    rospy.spin()

if __name__ == '__main__':
    run()