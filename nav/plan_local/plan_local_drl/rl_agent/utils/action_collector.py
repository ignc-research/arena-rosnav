#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from gym import spaces
import numpy as np

from tf.transformations import *



class ActionCollector():
    def __init__(self):
        self.v_max_ = 0.8 
        self.w_max_ = 1.2
        self.action_library={
            0: {"linear":0.0,           "angular": -self.w_max_},
            1: {"linear":self.v_max_,   "angular":0.0},
            2: {"linear":0.0,           "angular":self.w_max_},
            3: {"linear":self.v_max_,   "angular":self.w_max_ / 2},
            4: {"linear":self.v_max_,   "angular":-self.w_max_ / 2},
            5: {"linear":0.0,           "angular": 0.0}
        }
        self.N_DISCRETE_ACTIONS = len(self.action_library)
        self.action_space = spaces.Discrete(self.N_DISCRETE_ACTIONS)
    
    def get_action_space(self):
        return self.action_space  
     
    def get_cmd_vel(self, action_id):
        vel_msg = Twist()
        vel_msg.linear.x = self.action_library[action_id]["linear"]
        vel_msg.angular.z = self.action_library[action_id]["angular"]
        return vel_msg
    

if __name__ == '__main__':
    action_collector=ActionCollector()
    print(action_collector.get_cmd_vel(1))
    
    box = spaces.Box(low=3.0, high=4, shape=(2,2))
    print(box) 
    box.seed(4)
    for _ in range(1):
        print(box.sample())
    
    min_position = 0
    max_position = 10
    max_speed = 2
    goal_position = 0.5 
    low = np.array([min_position, -max_speed])
    high = np.array([max_position, max_speed])
    action_space = spaces.Discrete(3)  # action space
    observation_space = spaces.Box(low, high) # 
    print("*"*10)
    print(observation_space)
    for _ in range(2):
        print(observation_space.sample())
    
    observation_space=spaces.Tuple((
            spaces.Box(low=0, high=10, shape=(10,), dtype=np.float32),
            spaces.Box(low=-10, high=0, shape=(3+2,), dtype=np.float32) 
        ))
    print("2"*10)
    print(observation_space.sample())
    print(type(observation_space.sample()))

    reward=spaces.Discrete(4)
    print(type(reward.sample()))