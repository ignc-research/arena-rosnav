#!/usr/bin/env python
import numpy as np
import rospy
import sys
import curses
import time
from pedsim_msgs.msg import AgentStates

class Listener:
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        self.stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()


    def simulated_agents_callback(self, agent_states_msg):
        for i, agent_state in enumerate(agent_states_msg.agent_states):
            id = agent_state.id
            social_state = agent_state.social_state
            x = agent_state.pose.position.x
            y = agent_state.pose.position.y
            self.stdscr.addstr(i, 0, "id: {} state: {:20} x: {:2.2f} y: {:2.2f} ".format(id, social_state, x, y))
        self.stdscr.refresh()
        
    def listen(self):
        rospy.Subscriber("pedsim_simulator/simulated_agents", AgentStates, self.simulated_agents_callback)

        # spin() simply keeps python from exiting until this node is stopped
        try:
            rospy.spin()
        finally:
            curses.echo()
            curses.nocbreak()
            curses.endwin()


if __name__ == '__main__':
    listener = Listener()
    listener.listen()