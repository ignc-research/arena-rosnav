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
        self.stdscr.addstr(0, 0, "{} {:20} {:13} {:13} {:5} {:5} {:8} {:8} {:8} {:8} {:8} {:8}".format(
            "id",
            "state",
            "talking_to",
            "listening_to",
            "(x,",
            "y)",
            "desired",
            "social",
            "obstacle",
            "combined",
            "vel",
            "acc",
        ))
        for i, agent_state in enumerate(agent_states_msg.agent_states):
            id = agent_state.id
            social_state = agent_state.social_state
            x = agent_state.pose.position.x
            y = agent_state.pose.position.y
            desired_force = agent_state.forces.desired_force
            desired_force = np.array([desired_force.x, desired_force.y])
            desired_force_magnitude = np.linalg.norm(desired_force)
            social_force = agent_state.forces.social_force
            social_force = np.array([social_force.x, social_force.y])
            social_force_magnitude = np.linalg.norm(social_force)
            obstacle_force = agent_state.forces.obstacle_force
            obstacle_force = np.array([obstacle_force.x, obstacle_force.y])
            obstacle_force_magnitude = np.linalg.norm(obstacle_force)
            combined_forces = desired_force + social_force + obstacle_force
            combined_forces_magnitude = np.linalg.norm(combined_forces)
            talking_to_id = agent_state.talking_to_id
            listening_to_id = agent_state.listening_to_id
            vel = np.array([agent_state.twist.linear.x, agent_state.twist.linear.y])
            vel_magnitude = np.linalg.norm(vel)
            acc = np.array([agent_state.acceleration.x, agent_state.acceleration.y])
            acc_magnitude = np.linalg.norm(acc)
            self.stdscr.addstr(i+1, 0, "{} {:20} {:13} {:13} {:5.1f} {:5.1f} {:8.2f} {:8.2f} {:8.2f} {:8.2f} {:8.2f} {:8.2f}".format(
                id,
                social_state,
                talking_to_id,
                listening_to_id,
                x,
                y,
                desired_force_magnitude,
                social_force_magnitude,
                obstacle_force_magnitude,
                combined_forces_magnitude,
                vel_magnitude,
                acc_magnitude)
            )
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