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
import Gtr
import math
from torch.nn.utils.rnn import pack_sequence, pad_sequence
import torch
import numpy as np

# global variables for DQN
NUM_ACTIONS = 5
num_observations=362
SEQ_LENGTH=64
SEQ_LENGTH_MAX=300000
DISCOUNT_FACTOR=0.99

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

        #load NN
        self.model_name="dqn_agent_best_gtr_3_layer_l2.dat"
        self.net = Gtr.TransformerDqn(NUM_ACTIONS,num_observations)
        self.device=torch.device('cuda')
        self.net.to(torch.device('cuda'))        
        self.net.train(False)# set training mode to false to deactivate dropout layer
        if self.model_name != None:
            self.net.load_state_dict(torch.load(self.model_name, map_location=torch.device('cuda')))
        self.reset()
        # subs
        self.sub_pose = rospy.Subscriber('/odom',Odometry,self.cbPose)
        self.sub_global_goal = rospy.Subscriber('/goal',PoseStamped, self.cbGlobalGoal)
        self.sub_sub_subgoal = rospy.Subscriber('/plan_manager/subgoal',PoseStamped, self.cbSubGoal)
        self.sub_subgoal = rospy.Subscriber('/scan',LaserScan, self.cbScan)
        # pubs
        self.pub_pose_marker = rospy.Publisher('/arena_pose',Marker,queue_size=1)
        self.pub_twist = rospy.Publisher('/cmd_vel',Twist,queue_size=1) 

        self.nn_timer = rospy.Timer(rospy.Duration(0.01),self.cbComputeActionArena)
        self.control_timer = rospy.Timer(rospy.Duration(0.3),self.cbControl)


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
        v_p = msg.pose.pose.position
        # v_g = self.global_goal.pose.position
        v_g = self.sub_goal
        v_pg = np.array([v_g.x-v_p.x,v_g.y-v_p.y])
        v_pose = np.array([math.cos(self.psi),math.sin(self.psi)])
        angle = np.math.atan2(np.linalg.det([v_pose,v_pg]),np.dot(v_pose,v_pg))
        self.distance = np.linalg.norm(v_pg)
        self.deg_phi = math.degrees(angle)
        # quaternion
        q = msg.pose.pose.orientation
        self.psi = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)) # bounded by [-pi, pi]
        # pose
        self.pose = msg.pose
  

        # self.visualize_pose(msg.pose.pose.position,msg.pose.pose.orientation)

    def goalReached(self):
        # how far ?
        if self.distance > 0.3:
            return False
        else:
            self.reset()
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

    def reset(self):
        self.episode_frame=0
        self.last_action=-1
        self.last_reward=0.0
        self.tensor_state_buffer = torch.zeros(SEQ_LENGTH_MAX, num_observations,dtype=torch.float).to(torch.device('cuda'))

    def countNan(self,data):
        n=0
        for i in data:
            if np.isnan(i):
                n += 1
            else:
                print(n)
                return n
        print(n)
        return n
            
    def cbComputeActionArena(self,event):
        if not self.goalReached():
            # input           
            # pack goal position relative to robot
            self.angle = self.deg_phi    
            #distance = self.distance   
            #lidarscan
            sample = np.asanyarray(self.scan.ranges)
            # print(np.count_nonzero(~np.isnan(sample)))
            # print(self.countNan(sample))
            sample[np.isnan(sample)] = 3.5
            sample=sample.tolist()
            #print("s",len(sample))

            # print("===============================")
            
            # sample=np.ones([360,]).tolist()
            # print(sample)

            observation=[self.distance, self.angle]+sample
            #print("obser",len(observation))
            ##output NN
            # passing observation through net
            #q = None
            self.tensor_state_buffer[self.episode_frame] = torch.FloatTensor(observation)
            if self.episode_frame > SEQ_LENGTH-1:
                start_index= self.episode_frame-(SEQ_LENGTH-1)
                L=SEQ_LENGTH
            else:
                start_index = 0
                L=self.episode_frame+1
            state_v=[torch.narrow(self.tensor_state_buffer, dim=0, start=start_index, length=L)]
            t=pad_sequence(state_v).data.to(self.device)
            q = self.net(t)[-1,:]
            q=q.view(-1,NUM_ACTIONS)
            # select action with max q value
            _, act_v = torch.max(q, dim=1)
            action = int(act_v.item())
            self.last_action = action
            self.last_reward=q.squeeze()[action]
            self.update_action(action)
            self.episode_frame=self.episode_frame+1

        else:
            # print(self.global_goal.pose.position)
            # print("stop moving: "+str(self.distance))
            self.stop_moving()
            return

    def performAction(self, action):        
        action_space = {0: [0.2,0], 1: [0.15,0.35], 2: [0.15,-0.35], 3: [0.0,0.75], 4: [0.0,-0.75]}        
        twist = Twist()
        twist.linear.x = action_space[action][0]
        twist.angular.z = action_space[action][1]
        
        # if not self.goalReached():
        #     # print(actionLib[action][0])
        #     twist.linear.x = action_space[action][0]
        #     # if rotate only
        #     if action_space[action][0]==0:
        #         twist.angular.z = action_space[action][1]
        #     # otherewise check if angle error big enough
        #     elif abs(self.deg_phi)>5: 
        #         twist.angular.z = action_space[action][1]

        print("action "+str(action)+": "+str(action_space[action]))
        print("twist: "+str([twist.linear.x, twist.angular.z]))
        # print((sample))
        self.pub_twist.publish(twist)


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