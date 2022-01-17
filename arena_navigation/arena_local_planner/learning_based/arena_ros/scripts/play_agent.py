#!/usr/bin/env python3
# ros
import rospy
from observations.msg import Observation
from geometry_msgs.msg import (
    PoseStamped,
    Twist,
    Vector3,
    PoseWithCovarianceStamped,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# viz
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# arena
import fc2 as fc
import math
from torch.nn.utils.rnn import pack_sequence
import torch, rospkg
import numpy as np


class NN_tb3:
    def __init__(self):
        print("[play_agent]: in init")
        self.sub_obs = rospy.Subscriber(
            "/observation", Observation, self.cbObservation
        )
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def goalReached(self):
        # how far ?
        # if self.distance > 0.3:
        #     return False
        # else:
        #     return True
        goal_reached = rospy.get_param("/bool_goal_reached")

        if not goal_reached:
            return False
        else:
            return True

    def stop_moving(self):
        twist = Twist()
        # print(twist)
        self.pub_twist.publish(twist)

    def cbObservation(self, msg):
        # print("[Play_agent]: in cbObservation")
        if not self.goalReached():
            # NUM_ACTIONS = 5
            NUM_ACTIONS = 7
            num_observations = 362
            SEQ_LENGTH = 64
            SEQ_LENGTH_MAX = 300

            device = torch.device("cpu")
            net = fc.FC_DQN(num_observations, NUM_ACTIONS)
            net.train(False)
            # load NN

            current_dir_path = (
                rospkg.RosPack().get_path("arena_ros") + "/scripts/"
            )
            model_name = "advanced_agent2_best.dat"
            # model_name = "dqn_agent_best_fc_l2.dat"
            model_path = current_dir_path + model_name
            net.load_state_dict(torch.load(model_path, map_location=device))
            net.to(device)

            ##output NN
            # passing observation through net
            state_v = torch.FloatTensor([msg.observation]).to(device)
            q_vals_v = net(state_v)
            # select action with max q value
            _, act_v = torch.max(q_vals_v, dim=1)
            action = int(act_v.item())
            self.performAction(action)

        else:
            # print(self.global_goal.pose.position)
            # print("stop moving: "+str(self.distance))
            self.stop_moving()
            return

    def performAction(self, action):

        # action_space = {0: [0.2,0],1: [0.15,0.75],2: [0.15,-0.75],3: [0.0,1.5],4: [0.0,-1.5]}
        action_space = {
            0: [0.2, 0],
            1: [0.15, 0.75],
            2: [0.15, -0.75],
            3: [0.0, 1.5],
            4: [0.0, -1.5],
            5: [0.0, 0],
            6: [-0.1, 0],
        }
        # action_space = {0: [0.2,0], 1: [0.15,0.35], 2: [0.15,-0.35], 3: [0.0,0.75], 4: [0.0,-0.75]}
        # print(action)
        twist = Twist()
        twist.linear.x = action_space[action][0]
        twist.angular.z = action_space[action][1]

        print("action " + str(action) + ": " + str(action_space[action]))
        print("twist: " + str([twist.linear.x, twist.angular.z]))
        # print((sample))
        self.pub_twist.publish(twist)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down Node.")
        self.stop_moving()
        # rospy.loginfo("Stopped %s's velocity." %(self.veh_name))


def run():

    rospy.init_node("arena_tb3", anonymous=False)
    print(
        "==================================\narena node started\n=================================="
    )

    nn_tb3 = NN_tb3()
    rospy.on_shutdown(nn_tb3.on_shutdown)

    rospy.spin()


if __name__ == "__main__":
    run()
