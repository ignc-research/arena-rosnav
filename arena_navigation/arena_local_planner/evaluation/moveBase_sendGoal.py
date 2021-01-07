# goal
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy 
from geometry_msgs.msg import Vector3, Twist, PoseStamped
from nav_msgs.msg import Odometry

class newGoal():
    def __init__(self, x=0, y=0, z=0):

        self.curr_vel = Twist()
        self.mean_vel = []
        self.idle = False

        #subscriber
        self.sub_pose = rospy.Subscriber('/odom',Odometry,self.updateVel)
        #pub
        self.pub_goal =  rospy.Publisher('/goal',PoseStamped,queue_size=1)
        self.pub_mvb_goal =  rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)


        self.vel_timer = rospy.Timer(rospy.Duration(0.1),self.cbMeanVel)

    def updateVel(self,msg):
        self.curr_vel = msg.twist.twist

    def cbMeanVel(self,event):
        v = abs(self.curr_vel.linear.x) + abs(self.curr_vel.linear.y) + abs(self.curr_vel.angular.z)
        self.mean_vel.append(v)

        if len(self.mean_vel)>30:
            self.mean_vel.pop(0)
        
            if sum(self.mean_vel) > 0:
                self.idle = False  
            else:
                # none of the 30 vels > 0
                print "robot is in Idle Mode"
                self.idle = True
                self.mean_vel = []
            


    def send_goal(self,goal,movebase):
        # initialize goal
        print "goal received"
        pgoal = PoseStamped()
        pgoal.header.stamp = rospy.get_rostime()
        pgoal.header.frame_id = "map"
        pgoal.pose.position.x = goal.x
        pgoal.pose.position.y = goal.y
        pgoal.pose.orientation.z = goal.z
        pgoal.pose.orientation.w = 1
        # publish goal mode
        if movebase:
            self.pub_mvb_goal.publish(pgoal) 
        else:
            self.pub_goal.publish(pgoal) 

    def send_multiple_goals(self,goals,mode):
        n_goals = len(goals)
        n = 0
        while n < n_goals:
            if self.idle:
                curr_goal = goals[n]
                self.send_goal(curr_goal,mode)
                n += 1
                self.idle = False

    def on_shutdown(self):
        return

def run():
    print 'moveBase_sendGoal loaded'
    rospy.init_node('sendGoal',anonymous=False)
    # test goals
    mgoals = []
    # 1
    goal1 = Vector3()
    goal1.x = 22.6
    goal1.y = -0.743
    goal1.z = 0.97
    mgoals.append(goal1)
    # 2
    goal2 = Vector3()
    goal2.x = 9.8
    goal2.y = 13.86
    goal2.z = -0.75
    mgoals.append(goal2)
    # 3
    goal3 = Vector3()
    goal3.x = 20.33
    goal3.y = 15.47
    goal3.z = 0.9
    mgoals.append(goal3)
    # 4
    goal4 = Vector3()
    goal4.x = 0
    goal4.y = 0
    goal4.z = 0
    mgoals.append(goal4)

    sendGoals = newGoal()
    sendGoals.send_multiple_goals(mgoals,False)

    rospy.on_shutdown(sendGoals.on_shutdown)
    rospy.spin()

if __name__ == '__main__':
    run()