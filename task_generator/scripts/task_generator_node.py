#! /usr/bin/env python
import os, sys
sys.path.append('/home/junhui/study/Masterarbeit/arenarosnav/test_ws/src/arena_rosnav')
sys.path.append('/home/junhui/study/Masterarbeit/arenarosnav/test_ws/src/arena_rosnav/arena_navigation')
sys.path.append('/home/junhui/study/Masterarbeit/arenarosnav/test_ws/src/arena_rosnav/arena_navigation/arena_local_planner')
sys.path.append('/home/junhui/study/Masterarbeit/arenarosnav/test_ws/src/arena_rosnav/arena_navigation/arena_local_planner/learning_based/')
sys.path.append('/home/junhui/study/Masterarbeit/arenarosnav/test_ws/src/arena_rosnav/arena_navigation/arena_local_planner/learning_based//arena_local_planner_drl')
sys.path.append('/home/junhui/study/Masterarbeit/arenarosnav/test_ws/src/arena_rosnav/arena_navigation/arena_local_planner/learning_based//arena_local_planner_drl/scripts')
sys.path.append('/home/junhui/study/Masterarbeit/arenarosnav/test_ws/src/arena_rosnav/task_generator')
sys.path.append('/home/junhui/study/Masterarbeit/arenarosnav/test_ws/src/arena_rosnav/task_generator/task_generator')
from tasks import get_predefined_task
import rospy
from std_srvs.srv import Trigger, TriggerResponse
class TaskGenerator:
    def __init__(self):
        mode = rospy.get_param("task_mode")
        # print()
        # mode='random'
        self.task=get_predefined_task(mode)
        #declare new service task_generator, request are handled in callback task generate
        self.task_service_server= rospy.Service('task_generator', Trigger, self.callback_task_generate)
        
    def callback_task_generate(self,req):
        print("RESET")
        self.task.reset()
        return TriggerResponse(
            success= True,
            message= self.task.obstacles_manager.obstacle_name_str
        )
if __name__ == '__main__':
    rospy.init_node('task_generator')
    try:
        task_generator=TaskGenerator()
    except rospy.ROSInterruptException: pass    
    rospy.spin()    