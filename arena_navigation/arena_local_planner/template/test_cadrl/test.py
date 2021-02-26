from ros_nav import RosNav
import rospy
import copy
# cadrl
import rospkg
import network
import agent
import util
import numpy as np
from geometry_msgs.msg import PoseStamped

class TestNode():
    def __init__(self, nav, nn, actions):
        self.tb3 = nav
        self.desired_speed = 0.3

        # NN
        self.nn = nn
        self.actions = actions
        self.desired_position = PoseStamped()
        self.desired_action = np.zeros((2,))
    
        # control loop
        rospy.Timer(rospy.Duration(0.2),self.cbControl)
        rospy.Timer(rospy.Duration(0.01),self.cbComputeActionGA3C)
    
    def cbControl(self,event):
        print(self.desired_action[1]-self.tb3.angle_pose)
        print(self.desired_action[1])
        print(self.tb3.angle_pose)
        print("-----------------------")
        # print(self.tb3.angle_pose)
        # print(self.tb3.pose)
        # print(self.tb3.vel)
        # print(self.tb3.angle_pose)
        # print(self.tb3.sub_goal)

    def update_action(self, action):
        # print 'update action'
        self.desired_action = action
        self.desired_position.pose.position.x = self.tb3.pose.pose.position.x + 1*action[0]*np.cos(action[1])
        self.desired_position.pose.position.y = self.tb3.pose.pose.position.y + 1*action[0]*np.sin(action[1])

    def cbComputeActionGA3C(self,event):
        if not self.tb3.goalReached():
            ### construct agent_state
            x = self.tb3.pose.pose.position.x
            y = self.tb3.pose.pose.position.y
            v_x = self.tb3.vel.x
            v_y = self.tb3.vel.y
            radius = self.tb3.radius
            heading_angle = self.tb3.angle_pose
            pref_speed = self.desired_speed
            # next goal
            goal_x = self.tb3.sub_goal.x
            goal_y = self.tb3.sub_goal.y
            # norm vel if too fast
            v = np.linalg.norm(np.array([v_x, v_y]))
            if v > pref_speed:
                v_x = v_x * pref_speed / v
                v_y = v_y * pref_speed / v

            ### feed NN
            host_agent = agent.Agent(x, y, goal_x, goal_y, radius, pref_speed, heading_angle, 0)
            host_agent.vel_global_frame = np.array([v_x, v_y])

            other_agents_state = copy.deepcopy(self.tb3.obstacles_state)
            obs = host_agent.observe(other_agents_state)[1:]
            obs = np.expand_dims(obs, axis=0)

            predictions = self.nn.predict_p(obs)[0]

            raw_action = copy.deepcopy(self.actions[np.argmax(predictions)])
            action = np.array([pref_speed*raw_action[0], util.wrap(raw_action[1] + self.tb3.angle_pose)])
            self.update_action(action)
        else:
            action = [0, 0]
            self.update_action(action)

def run():
    
    # start node
    rospy.init_node("test", anonymous=False)
    # rospy.sleep(0.1) # sometimes node isnt recognized
    nav = RosNav('/goal','/plan_manager/subgoal')
    print('==================================\ntest-node started\n==================================')

    rospack = rospkg.RosPack()
    a = network.Actions()
    actions = a.actions
    # print(actions)
    num_actions = a.num_actions
    nn = network.NetworkVP_rnn(network.Config.DEVICE, 'network', num_actions)
    nn.simple_load(rospack.get_path('cadrl_ros')+'/checkpoints/network_01900000')

    test = TestNode(nav, nn, actions)
    rospy.spin()


if __name__ == '__main__':
    run()