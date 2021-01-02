#! /usr/bin/env python

import rospy


class RewardCollector():
    def __init__(self):
        pass

    def get_reward(self,action_id):
        reward=action_id
        return action_id