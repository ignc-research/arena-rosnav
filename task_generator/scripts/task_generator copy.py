#! /usr/bin/env python

from codecs import xmlcharrefreplace_errors
from logging import error
import time
import random
import math
import numpy as np
import yaml
from collections import OrderedDict
import move_base
import os
import rospy
import rospkg



#from tf.transformations import *
# import tf
from geometry_msgs.msg import TransformStamped

# delete model, spawn model, move model
from flatland_msgs.msg import Model
from flatland_msgs.srv import DeleteModel,DeleteModelRequest
from flatland_msgs.srv import SpawnModel,SpawnModelRequest
from flatland_msgs.srv import MoveModel,MoveModelRequest

# Step the flatland world
from flatland_msgs.srv import StepWorld,StepWorldRequest
from std_msgs.msg import Float64, Bool

# Map & Path
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from nav_msgs.srv import GetMap,GetMapRequest

# set postion
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped,PoseStamped
# set goal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
import actionlib

#sensor
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist, Point 
from std_srvs.srv import SetBool,Empty


class TaskGenerator():
    def __init__(self,robot_yaml_path:str):

        self.ROBOT_NAME = os.path.basename(robot_yaml_path).split('.')[0]
        with open(robot_yaml_path,"r") as fd:
            robot_info = yaml.load(fd)





        self.rate=rospy.Rate(100)
        self.ROBOT_RADIUS = robot_radius
        self.ROBOT_NAME=robot_name
        self.local_range=1.0
        
        # laser
        self._scan = LaserScan()
        self._flag_get_obs=False

        # model files
        self._flatland_models_path = rospkg.RosPack().get_path('simulator_setup')

        # map & freespace on static map
        self._map=OccupancyGrid()
        self._freespace=None

        # obstacles
        self._static_obstacles=[]
        self._dynamic_obstacles=[]
        self._peds=[]

        # goal,path,move_base_status
        self._global_path = Path()
        self._old_global_path_stamp=rospy.Time.now() # timestamp of the last global plan
        self._move_base_status = ""    # recent id of move_base status

        # services client
        self._service_client_get_map=rospy.ServiceProxy("%s/static_map" % self.NS,GetMap)
        self._service_client_move_robot_to = rospy.ServiceProxy('%s/move_model' % self.NS, MoveModel)
        self._service_client_delete_model = rospy.ServiceProxy('%s/delete_model' % self.NS, DeleteModel)
        self._service_client_spawn_model = rospy.ServiceProxy('%s/spawn_model' % self.NS, SpawnModel)
        self._sim_step = rospy.ServiceProxy('%s/step_world' % self.NS, StepWorld)

        
        # topic subscriber
        #self._map_sub=rospy.Subscriber("%s/map" % self.NS, OccupancyGrid, self._map_callback)
        self._global_path_sub = rospy.Subscriber("%s/move_base/NavfnROS/plan" % self.NS, Path, self._global_path_callback)

        self._goal_status_sub = rospy.Subscriber("%s/move_base/status" % self.NS, GoalStatusArray,
                                                  self.goal_status_callback, queue_size=1)
        
        self._obs_sub=rospy.Subscriber("%s/scan"% self.NS, LaserScan,self._get_obs_callback)

        # topic publisher
        self._initialpose_pub=rospy.Publisher('%s/initialpose' % self.NS, PoseWithCovarianceStamped, queue_size=1)
        self._goal_pub = rospy.Publisher('%s/move_base_simple/goal' % self.NS, PoseStamped, queue_size=1)
        
        # action client
        self._action_move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # tf 
        # self._tf_broadcaster = tf.TransformBroadcaster()
        # self._tf_listener=tf.TransformListener()
        
        # clear world

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    """
    1. load map
    2. freespace 
    3. current position, goal position, robot radius
    4. ask for path
    5. place static obstacle in the path
    6. place dynamic obstacle in the path
    7. place robot in the path
    8. place peds in the path
    """

    """ remove object """
    def remove_object(self,name):

        srv_request=DeleteModelRequest()
        srv_request.name=name
        rospy.wait_for_service('%s/delete_model' % self.NS)
        result=self._service_client_delete_model.call(srv_request)
        print(result)
        print("Deleted model:",name)

    def remove_objects(self,*group_names):
        rospy.Duration(1)
        topics=rospy.get_published_topics()
        for t in topics:
            topic_name=t[0]
            for group_name in group_names:
                if group_name in topic_name:
                    object_name=topic_name.split("/")[-1]
                    print(object_name)
                    self.remove_object(object_name)

    def remove_all_static_obstacles(self):
        self.remove_objects("stat_obj")
        self._static_obstacles=[]

    def remove_all_dynamic_obstacles(self):
        pass

    """ Spawn static obstacles """
    def spawn_object(self, model_yaml_path,group_name,index, x,y,theta):
        
        srv_request = SpawnModelRequest()
        srv_request.yaml_path=model_yaml_path #"%s/obstacles/%s"%(self._flatland_models_path,model_name)
        srv_request.name=group_name+"_%d" %index #"stat_obj_%d" % index
        srv_request.ns = self.NS
        
        pose=Pose2D()
        pose.x=x
        pose.y=y
        pose.theta=theta
        srv_request.pose=pose

        rospy.wait_for_service('%s/spawn_model' % self.NS)

        # try to call service 
        response=self._service_client_spawn_model.call(srv_request)

        if response.success==False: # if service not succeeds, do something and redo service 
            print(response.message)
            print("delete old object and try again: spawn object")
            self.remove_object(srv_request.name)
            self.spawn_static_obstacle(srv_request.name, index, x,y,theta)
        else:                       # if service succeeds, print success
            #print("sucess")
            obstacle=Model()
            obstacle.yaml_path=srv_request.yaml_path
            obstacle.name=srv_request.name
            obstacle.ns=srv_request.ns
            obstacle.pose=srv_request.pose
            self._static_obstacles.append(obstacle)
        
        return 

    def spawn_static_obstacle(self,model_name,index,x,y,theta):
        model_yaml_path="%s/obstacles/%s"%(self._flatland_models_path,model_name)
        group_name="stat_obj"
        self.spawn_object(model_yaml_path,group_name,index, x,y,theta)

    def spawn_random_static_obstacles(self,max_num_obstacles):
        if max_num_obstacles == 0:
            num_static_obstacles = 0
        else:
            num_static_obstacles = random.randint(1, max_num_obstacles)
        
        model_name="random.model.yaml"
        for index in range(0,num_static_obstacles):
            self.generate_random_static_obstacle_yaml(model_name)
            x, y, theta = self.get_random_pos_on_map(self._map,self._freespace)
            self.spawn_static_obstacle(model_name,index,x, y, theta)
        
    def generate_random_static_obstacle_yaml(self,model_name="random.model.yaml"):
        yaml_path="%s/obstacles/%s"%(self._flatland_models_path,model_name)
        
        type_list=["circle","polygon"]#
        min_obstacle_radius=0.5
        max_obstacle_radius=1.5
        min_obstacle_vert=3
        max_obstacle_vert=6

        # define body
        body={}
        body["name"]="random"
        body["pose"]=[0,0,0]
        body["type"]="dynamic"
        body["color"]=[1, 0.2, 0.1, 1.0] #[0.2, 0.8, 0.2, 0.75]
        body["footprints"]=[]

        # define footprint 
        n=random.randint(0,len(type_list))
        if n==len(type_list):
            n=n-1
        random_type=type_list[n]

        f={}
        f["type"]=random_type
        f["density"]=0
        f["layers"]=["all"]
        f["collision"]='true'
        f["sensor"]="false"

        if f["type"]=="circle":
            f["radius"]=random.uniform(min_obstacle_radius,max_obstacle_radius)
        elif f["type"]=="polygon":
            f["points"]=[]
            random_num_vert=random.randint(min_obstacle_vert,max_obstacle_vert)
            random_length=random.uniform(min_obstacle_radius,max_obstacle_radius)
            #radius_y=random.uniform(min_obstacle_radius,max_obstacle_radius)
            for i in range(random_num_vert):
                angle=2*math.pi*(float(i)/float(random_num_vert))
                vert=[math.cos(angle)*random_length, math.sin(angle)*random_length]
                #print(vert)
                #print(angle)
                f["points"].append(vert)
        
        body["footprints"].append(f)
        
        # define plugins
        plugin_tween={}
        plugin_tween["name"]="Tween"
        plugin_tween["type"]="Tween"
        plugin_tween["body"]="random"
        plugin_tween["delta"]=[5, 3, 3.141592]
        plugin_tween["duration"]="2.0"
        plugin_tween["mode"]="yoyo"
        plugin_tween["easing"]="quadraticInOut"

        # define dict_file
        dict_file = {'bodies':[body],"plugins":[]}
        with open(yaml_path, 'w') as file:
            documents = yaml.dump(dict_file, file)

    """ map & freespace & valid position check """
    def get_static_map(self):
        srv_request=GetMapRequest()
        rospy.wait_for_service('%s/static_map' % self.NS)
        try:
            self._map=self._service_client_get_map(srv_request).map
            nmap=np.array(self._map.data)
            self._freespace=np.where(nmap == 0)[0]

        except error as e:
            print('Get static map error:',e)
        
        return 

    def get_random_pos_on_map(self,map,freespace):
        def coordinate_pixel_to_meter(index_grid,map):

            # index_grid to column_grid, row_grid
            column=int(index_grid%map.info.width)
            row=int((index_grid-column)/map.info.width)

            # colum_grid, row_grid to x,y in meter
            x=column*map.info.resolution+map.info.origin.position.x   
            y=row*map.info.resolution+map.info.origin.position.y

            return x,y    
           
        # choose random index_grid in freespace
        index=random.randint(0,len(freespace))
        index_grid=freespace[index]
        x,y=coordinate_pixel_to_meter(index_grid,map)
        
        n=1
        # check the random pos is valid or not
        while(not self.is_pos_valid(x,y,map)):
            index=random.randint(0,len(freespace))
            index_grid=freespace[index]
            x,y=coordinate_pixel_to_meter(index_grid,map)
            n=n+1
            
        theta = random.uniform(-math.pi, math.pi) # in radius
        print(" found a random point in freespace, within number of trail: ",n)
        return x, y, theta

    def is_pos_valid(self,x,y,map):
        # in pixel
        cell_radius = int((self.ROBOT_RADIUS+0.1)/map.info.resolution)
        x_index = int((x-map.info.origin.position.x)/map.info.resolution)
        y_index = int((y-map.info.origin.position.y)/map.info.resolution)

        # check occupancy around (x_index,y_index) with cell_radius
        for i in range(x_index-cell_radius, x_index+cell_radius, 1):
            for j in range(y_index-cell_radius, y_index+cell_radius, 1):
                index = j * map.info.width + i
                if index>=len(map.data):
                    
                    return False
                try:
                    value=map.data[index]
                except IndexError:
                    print("IndexError: index: %d, map_length: %d"%(index, len(map.data)))
                    
                    return False
                if value!=0:
                    
                    return False
        return True

    """ Robot init position """
    def set_random_robot_pos(self):
        success = False
        x, y, theta=0.0,0.0,0.0
        while not success:
            x, y, theta = self.get_random_pos_on_map(self._map,self._freespace)
            self.set_robot_pos(x, y, theta)
            success=True
        return x, y, theta

    def set_robot_pos(self,x,y,theta):
        # call service move_model
        pose=Pose2D()
        pose.x=x
        pose.y=y
        pose.theta=theta

        srv_request=MoveModelRequest()
        srv_request.name=self.ROBOT_NAME
        srv_request.pose=pose

        
        # call service
        rospy.wait_for_service('%s/move_model'% self.NS)
        self._service_client_move_robot_to(srv_request)
            
        # publish robot position
        self.pub_initial_position(x,y,theta)
   
    def pub_initial_position(self, x, y, theta):
        """
        Publishing new initial position (x, y, theta) --> for localization
        :param x x-position of the robot
        :param y y-position of the robot
        :param theta theta-position of the robot
        """
        initpose = PoseWithCovarianceStamped()
        initpose.header.stamp = rospy.get_rostime()
        initpose.header.frame_id = "map"
        initpose.pose.pose.position.x = x
        initpose.pose.pose.position.y = y
        #quaternion = Quaternion(axis=[0, 0, 1], angle=theta)
        quaternion = tf.transformations.quaternion_from_euler(0,0,theta)
        initpose.pose.pose.orientation.x = quaternion[0]
        initpose.pose.pose.orientation.y = quaternion[1]
        initpose.pose.pose.orientation.z = quaternion[2]
        initpose.pose.pose.orientation.w = quaternion[3]
        
        initpose.pose.covariance = [0.0000,0.0,0.0,0.0,0.0,0.0,
                                          0.0,0.00000,0.0,0.0,0.0,0.0,
                                          0.0,0.0,0.00000,0.0,0.0,0.0,
                                          0.0,0.0,0.0,0.00000,0.0,0.0,
                                          0.0,0.0,0.0,0.0,0.00000,0.0,
                                          0.0,0.0,0.0,0.0,0.0,0.00000]

        
        self._initialpose_pub.publish(initpose)
        """
        while not self.ctrl_c:
            connections = self._initialpose_pub.get_num_connections()
            print("c=",connections)
            if connections > 0:
                self._initialpose_pub.publish(initpose)
                rospy.loginfo("initial pose Published")
                break
            else:
                self.rate.sleep()
        """
        return
    
    """ Robot goal position """
    def set_random_robot_goal(self):
        x, y, theta = self.get_random_pos_on_map(self._map,self._freespace)
        self.pub_robot_goal(x, y, theta)
        return x, y, theta

    def pub_robot_goal(self,x=1,y=0,theta=0):
        # save gloabl path stamp
        self._old_global_path_stamp=self._global_path.header.stamp

        # prepare goal
        goal = PoseStamped()
        goal.header.stamp = rospy.get_rostime()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y

        quaternion = tf.transformations.quaternion_from_euler(0,0,theta)
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        
        # continuous publish the goal until action server has accept the goal: method1 
        
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        
        while not self.ctrl_c:
            connections = self._goal_pub.get_num_connections()
            if connections > 1:
                self._goal_pub.publish(goal)
                rospy.loginfo("goal Published")
                break
            else:
                self.rate.sleep()
        # continuous publish the goal until action server has accept the goal: method2
        """
        while(self._move_base_status!=0):
            print("last_element =",self._move_base_status)
            print("status =",self._move_base_status)
            rate.sleep()
        """
        
    def pub_robot_goal_to_movebase(self,x=1,y=0,theta=0):
        def feedback_callback(feedback):    
            print('[Feedback] planing')
        
        # prepare goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.get_rostime()#print(rospy.Time.now())
        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0,0,theta)
        print(quaternion)
        print(tf.transformations.euler_from_quaternion(quaternion))
        
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        # wait for server connection
        rospy.loginfo('Waiting for action Server: '+"/move_base")
        self._action_move_base_client.wait_for_server()
        rospy.loginfo('Action Server Found...'+"/move_base")
        # send goal
        self._action_move_base_client.send_goal(goal,feedback_cb=feedback_callback)

        """
        # monitor state
        state_result = self._action_move_base_client.get_state()
        for i in range(100):
            print(i)
            if 1>50:
                self._action_move_base_client.cancel_goal()
                break
        """

        """
        wait = self._action_move_base_client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self._action_move_base_client.get_result()
        """

    def goal_status_callback(self, data):
        """
        Recovery method for stable learning:
        Checking goal status callback from global planner.
        If goal is not valid, new goal will be published.
        :param status_callback
        """
        if len(data.status_list) > 0:
            last_element = data.status_list[-1]
            self._move_base_status=last_element.status
            #print(last_element.goal_id.id)
            #print(last_element.status)
        
    """global path"""
    def _global_path_callback(self,data):
        self._global_path=data
    
    def check_is_new_path_available(self):
        """waiting for path to be published"""
        is_available=False
        begin=time.time()
        timeout=0.1 # realtime 0.1 second
        
        # check if exsit new global path within timeout time
        while(time.time()-begin)<timeout:
            if self._global_path.header.stamp<=self._old_global_path_stamp or len(self._global_path.poses)==0:
                time.sleep(0.0001)
            else:
                is_available = True
                self._old_global_path_stamp = self._global_path.header.stamp
                break
        
        if not is_available:
            print("cannot get new global path")
        
        return is_available

    def generate_waypoints(self):
        pass

    """Tasks"""
    def set_task(self):
        # load static cost map
        task.get_static_map()
        
        #clear objects
        rospy.loginfo("remove all static obstacles")
        self.remove_all_static_obstacles()
        

        # set robot init position
        rospy.loginfo("set init robot pos")
        self.set_random_robot_pos()
       

        # add objects
        rospy.loginfo("spawn obstacles")
        self.spawn_random_static_obstacles(max_num_obstacles=10)
        

        # set robot goal position
        rospy.loginfo("set random goal")
        self.set_random_robot_goal()
        
        while((self._move_base_status==1) | (self._move_base_status==0)):
            self.rate.sleep()

        print(self._move_base_status)   
        if self._move_base_status==3:
            rospy.loginfo("succeed to reach the goal")
            return True
        else:
            rospy.loginfo("failed to reach the goal")
            return False
    
    """ Test purpose """
    def tf_listen_map_to_odom(self):
        # listen tf map->odom
        self._tf_listener.waitForTransform("map","odom",rospy.Time(0),rospy.Duration(3.0))
        (trans,rot)=self._tf_listener.lookupTransform("map","odom",rospy.Time(0))
        print(trans)
        
        # publish tf map->odom
        for i in range(1,1):
            translation=(10,2,0)
            rotation=rot
            time=rospy.Time.now()
            child="odom"
            parent="map"
            self._tf_broadcaster.sendTransform(translation, rotation, time, child, parent)
            
    """shutdown"""
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    """Step"""
    def take_sim_step(self):
        """
        
        """
        msg = StepWorldRequest()
        rospy.wait_for_service('%s/step_world' % self.NS)
        ret=self._sim_step(msg)
        #print("result=",ret)
        return


    def _get_obs_callback(self,data):
        #print(dir(data.header))
        #print(dir(data.header.stamp))
        print(data.header.stamp.to_sec())
        self._flag_get_obs=True


if __name__ == '__main__':

    rospy.init_node('Task', anonymous=True)
    ns=""
    robot_radius=0.5
    robot_name="myrobot"
    task=TaskGenerator(ns,robot_name,robot_radius)
    
   
    
    """Test1: running task"""
    for n in range(10):
        print("-------------------------------------------------")
        print( "Task episode", n)
        print("-------------------------------------------------")
        task.set_task()
    
    
    """Test2: simulator step service"""
    """
    print("a")
    i=0
    start=time.time()
    r=rospy.Rate(10)
    while(task._flag_get_obs==False):
        i=i+1
      
        task.take_sim_step()
        if(task._flag_get_obs==True):
     
            task._flag_get_obs=False
    """
        

        
    
    
    

    
    
    
    
    
    
    

