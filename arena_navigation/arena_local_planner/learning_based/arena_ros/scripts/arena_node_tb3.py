#!/usr/bin/env python
import rospy
import gru
from torch.nn.utils.rnn import pack_sequence
import torch
import numpy as np

class NN_tb3():
    def __init__(self):

        #parameters
        self.NUM_ACTIONS = 5
        self.num_observations=74
        self.SEQ_LENGTH=64
        self.SEQ_LENGTH_MAX=300

        self.episode_idx=0
        self.last_action=-1
        self.tensor_state_buffer = torch.zeros(self.SEQ_LENGTH_MAX, self.num_observations,dtype=torch.float)
      
        # self.control_timer = rospy.Timer(rospy.Duration(0.01),self.cbControl)
        self.nn_timer = rospy.Timer(rospy.Duration(0.1),self.cbComputeActionGA3C)

    def cbGlobalGoal(self,msg):
        self.stop_moving_flag = True
        self.new_global_goal_received = True
        self.global_goal = msg
        self.operation_mode.mode = self.operation_mode.SPIN_IN_PLACE
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
        self.cbVel(msg)
        self.num_poses += 1
        q = msg.pose.pose.orientation
        self.psi = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)) # bounded by [-pi, pi]
        self.pose = msg.pose
        self.visualize_pose(msg.pose.pose.position,msg.pose.pose.orientation)

    def cbVel(self, msg):
        self.vel = msg.twist.twist.linear

    def update_action(self, action):
        # print 'update action'
        self.desired_action = action
        self.desired_position.pose.position.x = self.pose.pose.position.x + 1*action[0]*np.cos(action[1])
        self.desired_position.pose.position.y = self.pose.pose.position.y + 1*action[0]*np.sin(action[1])
    
    def step(self,observation, h, net):
        # passing observation through net
        q = None
        self.tensor_state_buffer[self.episode_idx] = torch.FloatTensor(observation);
        if self.episode_idx > self.SEQ_LENGTH-1:
            start_index= self.episode_idx-(self.SEQ_LENGTH-1)
            L=self.SEQ_LENGTH
        else:
            start_index = 0
            L=self.episode_idx+1
        state_v=[torch.narrow(self.tensor_state_buffer, dim=0, start=start_index, length=L)]
        t=pack_sequence(state_v, enforce_sorted=False)
        q,_ = net(t,h)
        q=q.view(-1,self.NUM_ACTIONS)
        # select action with max q value
        _, act_v = torch.max(q, dim=1)
        action = int(act_v.item())
        last_action = action
        return action

    def cbControl(self, event):

        if self.goal.header.stamp == rospy.Time(0) or self.stop_moving_flag and not self.new_global_goal_received:
            self.stop_moving()
            return
        elif self.operation_mode.mode==self.operation_mode.NN:
            desired_yaw = self.desired_action[1]
            yaw_error = desired_yaw - self.psi
            if abs(yaw_error) > np.pi:
                yaw_error -= np.sign(yaw_error)*2*np.pi

            gain = 1.3 # canon: 2
            vw = gain*yaw_error

            use_d_min = True
            if False: # canon: True
                # use_d_min = True
                # print "vmax:", self.find_vmax(self.d_min,yaw_error)
                vx = min(self.desired_action[0], self.find_vmax(self.d_min,yaw_error))
            else:
                vx = self.desired_action[0]
      
            twist = Twist()
            twist.angular.z = vw
            twist.linear.x = vx
            self.pub_twist.publish(twist)
            self.visualize_action(use_d_min)
            return

        elif self.operation_mode.mode == self.operation_mode.SPIN_IN_PLACE:
            print('Spinning in place.')
            self.stop_moving_flag = False
            angle_to_goal = np.arctan2(self.global_goal.pose.position.y - self.pose.pose.position.y, \
                self.global_goal.pose.position.x - self.pose.pose.position.x) 
            global_yaw_error = self.psi - angle_to_goal
            if abs(global_yaw_error) > 0.5:
                vx = 0.0
                vw = 1.0
                twist = Twist()
                twist.angular.z = vw
                twist.linear.x = vx
                self.pub_twist.publish(twist)
                # print twist
            else:
                print('Done spinning in place')
                self.operation_mode.mode = self.operation_mode.NN
                # self.new_global_goal_received = False
            return
        else:
            self.stop_moving()
            return

    def cbComputeActionGA3C(self, event):            
        # Input
        # pack goal position relative to robot
        angle = 0      #in rad
        distance = 0    #in meter

        #  laser samples
        ##lidarscan
        ##hier the length of lidascan is 72 or 360,which would be noticed on the name of the model.

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

        # if host_agent.dist_to_goal < 2.0: # and self.percentComplete>=0.9:
        #     # print "somewhat close to goal"
        #     pref_speed = max(min(kp_v * (host_agent.dist_to_goal-0.1), pref_speed), 0.0)
        #     action[0] = min(raw_action[0], pref_speed)
        #     turn_amount = max(min(kp_r * (host_agent.dist_to_goal-0.1), 1.0), 0.0) * raw_action[1]
        #     action[1] = util.wrap(turn_amount + self.psi)
        # if host_agent.dist_to_goal < 0.3:
        #     # current goal, reached, increment for next goal
        #     print("===============\ngoal reached: "+str([goal_x, goal_y]))
        #     self.stop_moving_flag = True
        #     self.new_global_goal_received = False
        #     self.stop_moving()
        #     # self.goal_idx += 1
        # else:
        #     self.stop_moving_flag = False

        # self.update_action(action)

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