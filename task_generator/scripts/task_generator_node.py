#! /usr/bin/env python

from logging import setLogRecordFactory
import rospy
import time
from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.msg import Odometry
from task_generator.tasks import get_predefined_task
from std_msgs.msg import Int16
# for clearing costmap
from clear_costmap import clear_costmaps
class TaskGenerator:
    def __init__(self):
        #
        self.sr = rospy.Publisher('/scenario_reset', Int16, queue_size=1)
        self.nr = 0
        mode = rospy.get_param("~task_mode")
        
        
        scenarios_json_path = rospy.get_param("~scenarios_json_path")
       
        paths = {"scenario": scenarios_json_path}
  
        self.task = get_predefined_task("",mode, PATHS=paths)
       


        # if auto_reset is set to true, the task generator will automatically reset the task
        # this can be activated only when the mode set to 'ScenarioTask'
        auto_reset = rospy.get_param("~auto_reset")

        # if the distance between the robot and goal_pos is smaller than this value, task will be reset
        self.timeout_= rospy.get_param("~timeout")
        self.timeout_= self.timeout_*60             # sec
        self.start_time_=time.time()                # sec
        self.delta_ = rospy.get_param("~delta")
        robot_odom_topic_name = rospy.get_param(
            "robot_odom_topic_name", "odom")
        
        auto_reset = auto_reset and mode == "scenario"
        self.curr_goal_pos_ = None
        
        
        if auto_reset:
            rospy.loginfo(
                "Task Generator is set to auto_reset mode, Task will be automatically reset as the robot approaching the goal_pos")
            self.reset_task()
            self.robot_pos_sub_ = rospy.Subscriber(
                robot_odom_topic_name, Odometry, self.check_robot_pos_callback)

            rospy.Timer(rospy.Duration(0.5),self.goal_reached)
            
        else:
            # declare new service task_generator, request are handled in callback task generate
            self.reset_task()
            # self.task_generator_srv_ = rospy.Service(
            #     'task_generator', Empty, self.reset_srv_callback)
                
        self.err_g = 100
        


    def goal_reached(self,event):

        if self.err_g < self.delta_:
            print(self.err_g)
            self.reset_task()
        if(time.time()-self.start_time_>self.timeout_):
            print("timeout")
            self.reset_task()


    # def clear_costmaps(self):
    #     bashCommand = "rosservice call /move_base/clear_costmaps"
    #     process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    #     output, error = process.communicate()
    #     print([output, error])

    def reset_srv_callback(self, req):
        rospy.loginfo("Task Generator received task-reset request!")
        self.task.reset()
        return EmptyResponse()


    def reset_task(self):
        self.start_time_=time.time()
        info = self.task.reset()
        
        # clear_costmaps()
        if info is not None:
            self.curr_goal_pos_ = info['robot_goal_pos']
        rospy.loginfo("".join(["="]*80))
        rospy.loginfo("goal reached and task reset!")
        rospy.loginfo("".join(["="]*80))
        self.sr.publish(self.nr)
        self.nr += 1

    def check_robot_pos_callback(self, odom_msg: Odometry):
        robot_pos = odom_msg.pose.pose.position
        robot_x = robot_pos.x
        robot_y = robot_pos.y
        goal_x = self.curr_goal_pos_[0]
        goal_y = self.curr_goal_pos_[1]

        self.err_g = (robot_x-goal_x)**2+(robot_y-goal_y)**2
           


if __name__ == '__main__':
    rospy.init_node('task_generator')
    task_generator = TaskGenerator()
    rospy.spin()