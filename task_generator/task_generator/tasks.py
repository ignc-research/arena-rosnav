import os
from abc import ABC, abstractmethod
from threading import Condition, Lock
import rospy
import rospkg
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Pose2D
from rospy.exceptions import ROSException
from obstacles_manager import ObstaclesManager
from robot_manager import RobotManager
import json


class ABSTask(ABC):
    """An abstract class, all tasks must implement reset function.

    """

    def __init__(self, obstacles_manager: ObstaclesManager, robot_manager: RobotManager):
        self.obstacles_manager = obstacles_manager
        self.robot_manager = robot_manager
        self._service_client_get_map = rospy.ServiceProxy("static_map", GetMap)
        self._map_lock = Lock()
        rospy.Subscriber("map", OccupancyGrid, self._update_map)
        # a mutex keep the map is not unchanged during reset task.

    @abstractmethod
    def reset(self):
        """
        a funciton to reset the task. Make sure that _map_lock is used.
        """

    def _update_map(self, map_: OccupancyGrid):
        with self._map_lock:
            self.obstacles_manager.update_map(map_)
            self.robot_manager.update_map(map_)


class RandomTask(ABSTask):
    """ Evertime the start position and end position of the robot is reset.
    """

    def __init__(self, obstacles_manager: ObstaclesManager, robot_manager: RobotManager):
        super().__init__(obstacles_manager, robot_manager)

    def reset(self):
        """[summary]
        """
        with self._map_lock:
            max_fail_times = 3
            fail_times = 0
            while fail_times < max_fail_times:
                try:
                    start_pos, goal_pos = self.robot_manager.set_start_pos_goal_pos()
                    self.obstacles_manager.reset_pos_obstacles_random(
                        forbidden_zones=[
                            (start_pos.x,
                                start_pos.y,
                                self.robot_manager.ROBOT_RADIUS),
                            (goal_pos.x,
                                goal_pos.y,
                                self.robot_manager.ROBOT_RADIUS)])
                    break
                except rospy.ServiceException as e:
                    rospy.logwarn(repr(e))
                    fail_times += 1
            if fail_times == max_fail_times:
                raise Exception("reset error!")


class ManualTask(ABSTask):
    """randomly spawn obstacles and user can mannually set the goal postion of the robot
    """

    def __init__(self, obstacles_manager: ObstaclesManager, robot_manager: RobotManager):
        super().__init__(obstacles_manager, robot_manager)
        # subscribe
        rospy.Subscriber("manual_goal", Pose2D, self._set_goal_callback)
        self._goal = Pose2D()
        self._new_goal_received = False
        self._manual_goal_con = Condition()

    def reset(self):
        while True:
            with self._map_lock:
                self.obstacles_manager.reset_pos_obstacles_random()
                self.robot_manager.set_start_pos_random()
                with self._manual_goal_con:
                    # the user has 60s to set the goal, otherwise all objects will be reset.
                    self._manual_goal_con.wait_for(
                        self._new_goal_received, timeout=60)
                    if not self._new_goal_received:
                        raise Exception(
                            "TimeOut, User does't provide goal position!")
                    else:
                        self._new_goal_received = False
                    try:
                        # in this step, the validation of the path will be checked
                        self.robot_manager.publish_goal(
                            self._goal.x, self._goal.y, self._goal.theta)
                    except Exception as e:
                        rospy.logwarn(repr(e))

    def _set_goal_callback(self, goal: Pose2D):
        with self._manual_goal_con:
            self._goal = goal
            self._new_goal_received = True
        self._manual_goal_con.notify()


class StagedRandomTask(RandomTask):
    def __init__(self, obstacles_manager: ObstaclesManager, robot_manager: RobotManager, start_stage: int = 0, PATHS = None):
        super().__init__(obstacles_manager, robot_manager)
        self.curr_stage = start_stage
        self.json_location = os.path.join(PATHS.get('model'), "hyperparameters.json")
        self._initiate_stage()
        
    def next_stage(self):
        self.curr_stage += 1
        self._update_curr_stage_json()
        self._remove_obstacles()
        self._initiate_stage()

    def _initiate_stage(self):
        if self.curr_stage == 0:
            self.obstacles_manager.register_random_obstacles(0, 0.0)
        elif self.curr_stage == 1:
            self.obstacles_manager.register_random_obstacles(10, 0.0)
        elif self.curr_stage == 2:
            self.obstacles_manager.register_random_obstacles(20, 0.0)
        elif self.curr_stage == 3:
            self.obstacles_manager.register_random_obstacles(10, 1)
        elif self.curr_stage == 4:
            self.obstacles_manager.register_random_obstacles(20, 0.4)
        else:
            self.obstacles_manager.register_random_obstacles(30, 0.4)

    def _update_curr_stage_json(self):
        if os.path.isfile(self.json_location):
            with open(self.json_location, "r") as file:
                hyperparams = json.load(file)
            try:
                hyperparams['curr_stage'] = self.curr_stage
            except Exception:
                raise Warning("Parameter 'curr_stage' not found in 'hyperparameters.json'!")
            else:
                with open(self.json_location, "w", encoding='utf-8') as target:
                    json.dump(hyperparams, target, ensure_ascii=False, indent=4)
        else:
            raise Warning("File not found %s" % self.json_location)
    
    def _remove_obstacles(self):
        self.obstacles_manager.remove_obstacles()


def get_predefined_task(mode="random", start_stage: int = 0, PATHS: dict = None):

    # TODO extend get_predefined_task(mode="string") such that user can choose between task, if mode is

    # check is it on traininig mode or test mode. if it's on training mode
    # flatland will provide an service called 'step_world' to change the simulation time
    # otherwise it will be bounded to real time.
    try:
        rospy.wait_for_service('step_world', timeout=0.5)
        TRAINING_MODE = True
    except ROSException:
        TRAINING_MODE = False

    if TRAINING_MODE:
        from flatland_msgs.srv import StepWorld, StepWorldRequest
        # This is kind of hacky. the services provided by flatland may take a couple of step to complete
        # the configuration including the map service.
        steps = 400
        step_world = rospy.ServiceProxy(
            'step_world', StepWorld, persistent=True)
        for _ in range(steps):
            step_world()

    # get the map
    service_client_get_map = rospy.ServiceProxy("static_map", GetMap)
    map_response = service_client_get_map()

    # use rospkg to get the path where the model config yaml file stored
    models_folder_path = rospkg.RosPack().get_path('simulator_setup')
    # robot's yaml file is needed to get its radius.
    robot_manager = RobotManager(map_response.map, os.path.join(
        models_folder_path, 'robot', "myrobot.model.yaml"), TRAINING_MODE)

    obstacles_manager = ObstaclesManager(map_response.map, TRAINING_MODE)
    # only generate 3 static obstaticles
    # obstacles_manager.register_obstacles(3, os.path.join(
    # models_folder_path, "obstacles", 'random.model.yaml'), 'static')
    # generate 5 static or dynamic obstaticles
    # obstacles_manager.register_random_obstacles(20, 0.4)

    # TODO In the future more Task will be supported and the code unrelated to
    # Tasks will be moved to other classes or functions.
    if mode == "random":
        obstacles_manager.register_random_obstacles(20, 0.4)
        task = RandomTask(obstacles_manager, robot_manager)
        print("random tasks requested")
    if mode == "manual":
        obstacles_manager.register_random_obstacles(20, 0.4)
        task = ManualTask(obstacles_manager, robot_manager)
        print("manual tasks requested")
    if mode == "staged":
        task = StagedRandomTask(obstacles_manager, robot_manager, start_stage, PATHS)
    #task = RandomTask(obstacles_manager, robot_manager)

    return task
