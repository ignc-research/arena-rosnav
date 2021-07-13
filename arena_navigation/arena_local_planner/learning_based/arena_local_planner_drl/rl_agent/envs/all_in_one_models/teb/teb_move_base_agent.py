from collections import deque

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from rl_agent.envs.all_in_one_models.model_base_class import ModelBase


class TebMoveBaseAgent(ModelBase):

    def __init__(self, name: str, ns_prefix: str):
        super().__init__(dict(), name)
        self._ns_prefix = ns_prefix
        self._last_cmd_vel = Twist()
        self._cmd_vel_deque = deque()
        self.max_deque_size = 10

        self._robot_state_sub = rospy.Subscriber(
            f'{self._ns_prefix}cmd_vel_' + name, Twist, self._callback_cmd_vel, tcp_nodelay=True)

    def get_next_action(self, observation_dict: dict) -> np.ndarray:
        return np.array([self._last_cmd_vel.linear.x, self._last_cmd_vel.angular.z])

    def wait_for_agent(self):
        return True

    def reset(self):
        pass

    def _callback_cmd_vel(self, msg_twist):
        self._last_cmd_vel = msg_twist