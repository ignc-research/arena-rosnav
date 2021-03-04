#!/usr/bin/env python
import rospy
import time

from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

class ActionPublisher():
    def __init__(self):
        if rospy.get_param("train_mode"):
            raise Exception("This node should be used solely in eval mode!")

        rospy.init_node('action_publisher', anonymous = True)

        self._step_size = rospy.get_param("step_size")
        self._update_rate = rospy.get_param("update_rate")
        # real time second in sim time
        self._real_second_in_sim = self._step_size * self._update_rate
        self._action_publish_rate = rospy.get_param("/robot_action_rate")

        ns_prefix = "/sim_1/"
        self._pub = rospy.Publisher(
            f"{ns_prefix}cmd_vel", Twist, queue_size=1)
        self._sub = rospy.Subscriber(
            f"{ns_prefix}cmd_vel_pub", Twist, self.callback_receive_cmd_vel, queue_size=1)
        # self._clock_sub = rospy.Subscriber(
        #     f"{ns_prefix}clock", Clock, self.callback_clock)
        
        self._action = Twist()
        self._clock = Clock().clock.to_sec()

        # apply rate in sim time
        rate = (1/self._action_publish_rate)/self._real_second_in_sim

        last = 0
        while not rospy.is_shutdown():
            self._pub.publish(self._action)
            # print(f"sim time between cmd_vel: {self._clock - last}")
            # last = self._clock

    def callback_receive_cmd_vel(self, msg_cmd_vel: Twist):
        self._action = msg_cmd_vel

    def callback_clock(self, msg_clock: Clock):
        self._clock = msg_clock.clock.to_sec()
        
if __name__ == '__main__':
    try:
        ActionPublisher()
    except rospy.ROSInterruptException:
        pass
