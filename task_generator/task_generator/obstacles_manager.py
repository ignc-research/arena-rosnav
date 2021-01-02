import math
import random
from typing import Union
import re
import yaml
import os
from flatland_msgs.srv import DeleteModel, DeleteModelRequest
from flatland_msgs.srv import SpawnModel, SpawnModelRequest
from flatland_msgs.srv import MoveModel, MoveModelRequest
from flatland_msgs.srv import StepWorld
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D
import rospy
import rospkg
from .utils import generate_freespace_indices, get_random_pos_on_map


class ObstaclesManager:
    """
    A manager class using flatland provided services to spawn, move and delete obstacles.
    """

    def __init__(self, map_: OccupancyGrid,is_training = True):

        # setup proxy to handle  services provided by flatland
        rospy.wait_for_service('move_model', timeout=20)
        rospy.wait_for_service('delete_model', timeout=20)
        rospy.wait_for_service('spawn_model', timeout=20)
        if is_training:
            rospy.wait_for_service('step_world', timeout=20)
        # allow for persistent connections to services
        self._srv_move_model = rospy.ServiceProxy(
            'move_model', MoveModel, persistent=True)
        self._srv_delete_model = rospy.ServiceProxy(
            'delete_model', DeleteModel, persistent=True)
        self._srv_spawn_model = rospy.ServiceProxy(
            'spawn_model', SpawnModel, persistent=True)
        #self._srv_sim_step = rospy.ServiceProxy('step_world', StepWorld, persistent=True)

        self.update_map(map_)
        self.obstacle_name_list = []
        self._obstacle_name_prefix = 'obstacles'
        # remove all existing obstacles generated before create an instance of this class
        self.remove_obstacles()

    def update_map(self, new_map: OccupancyGrid):
        self.map = new_map
        # a tuple stores the indices of the non-occupied spaces. format ((y,....),(x,...)
        self._free_space_indices = generate_freespace_indices(self.map)

    def register_obstacles(self, num_obstacles: int, model_yaml_file_path: str, type_obstacle: str):
        """register the static obstacles and request flatland to respawn the them.

        Args:
            num_obstacles (string): the number of the obstacle instance to be created.
            model_yaml_file_path (string or None): model file path. it must be absolute path! 
            type_obstacle (string or None): type of the obstacle. it must be 'dynamic' or 'static'. 


        Raises:
            Exception:  Rospy.ServiceException(f" failed to register obstacles")

        Returns:
            self.
        """
        assert os.path.isabs(
            model_yaml_file_path), "The yaml file path must be absolute path, otherwise flatland can't find it"
        assert type_obstacle == 'dynamic' or type_obstacle == 'static', 'The type of the obstacle must be dynamic or static'
        # the name of the model yaml file have the format {model_name}.model.yaml
        type_obstacle = self._obstacle_name_prefix+'_'+type_obstacle
        model_name = os.path.basename(model_yaml_file_path).split('.')[0]
        name_prefix = type_obstacle + '_' + model_name
        count_same_type = sum(
            1 if obstacle_name.startswith(name_prefix) else 0
            for obstacle_name in self.obstacle_name_list)

        for instance_idx in range(count_same_type, count_same_type + num_obstacles):
            max_num_try = 2
            i_curr_try = 0
            while i_curr_try < max_num_try:
                spawn_request = SpawnModelRequest()
                spawn_request.yaml_path = model_yaml_file_path
                spawn_request.name = f'{name_prefix}_{instance_idx:02d}'
                spawn_request.ns = rospy.get_namespace()
                # x, y, theta = get_random_pos_on_map(self._free_space_indices, self.map,)
                # set the postion of the obstacle out of the map to hidden them
                x = self.map.info.origin.position.x - 3 * \
                    self.map.info.resolution * self.map.info.height
                y = self.map.info.origin.position.y - 3 * \
                    self.map.info.resolution * self.map.info.width
                theta = theta = random.uniform(-math.pi, math.pi)
                spawn_request.pose.x = x
                spawn_request.pose.y = y
                spawn_request.pose.theta = theta
                # try to call service
                response = self._srv_spawn_model.call(spawn_request)
                if not response.success:  # if service not succeeds, do something and redo service
                    rospy.logwarn(
                        f"spawn object {spawn_request.name} failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
                    rospy.logwarn(response.message)
                    i_curr_try += 1
                else:
                    self.obstacle_name_list.append(spawn_request.name)
                    break
            if i_curr_try == max_num_try:
                raise rospy.ServiceException(f" failed to register obstacles")
        return self

    def register_random_obstacles(self, num_obstacles: int):
        """register static or dynamic obstacles with random shape

        Args:
            num_obstacles (int):
        """
        # since flatland  can only config the model by parsing the yaml file, we need to create a file for every random obstacle
        tmp_folder = os.path.join(rospkg.RosPack().get_path(
            'simulator_setup'), 'tmp_random_obstacles')
        os.makedirs(tmp_folder, exist_ok=True)
        for _ in range(num_obstacles):
            tmp_model_name = 'random.model.yaml'
            obstacle_type = self.generate_random_static_obstacle_yaml(
                tmp_folder, tmp_model_name)
            self.register_obstacles(1, os.path.join(
                tmp_folder, tmp_model_name), obstacle_type)
        # remove the tmp folder
        import shutil
        shutil.rmtree(tmp_folder)

    def move_obstacle(self, obstacle_name: str, x: float, y: float, theta: float):
        """move the obstacle to a given position

        Args:
            obstacle_name (str): [description]
            x (float): [description]
            y (float): [description]
            theta (float): [description]
        """

        assert obstacle_name in self.obstacle_name_list, "can't move the obstacle because it has not spawned in the flatland"
        # call service move_model

        srv_request = MoveModelRequest()
        srv_request.name = obstacle_name
        srv_request.pose.x = x
        srv_request.pose.y = y
        srv_request.pose.theta = theta

        self._srv_move_model(srv_request)

    def reset_pos_obstacles_random(self, active_obstacle_rate: float = 1, forbidden_zones: Union[list, None] = None):
        """randomly set the position of all the obstacles. In order to dynamically control the number of the obstacles within the
        map while keep the efficiency. we can set the parameter active_obstacle_rate so that the obstacles non-active will moved to the
        outside of the map

        Args:
            active_obstacle_rate (float): a parameter change the number of the obstacles within the map
            forbidden_zones (list): a list of tuples with the format (x,y,r),where the the obstacles should not be reset.
        """
        active_obstacle_names = random.sample(self.obstacle_name_list, int(
            len(self.obstacle_name_list) * active_obstacle_rate))
        non_active_obstacle_names = set(
            self.obstacle_name_list) - set(active_obstacle_names)

        # non_active obstacles will be moved to outside of the map
        resolution = self.map.info.resolution
        pos_non_active_obstacle = Pose2D()
        pos_non_active_obstacle.x = self.map.info.origin.position.x - \
            resolution * self.map.info.width
        pos_non_active_obstacle.y = self.map.info.origin.position.y - \
            resolution * self.map.info.width

        for obstacle_name in active_obstacle_names:
            move_model_request = MoveModelRequest()
            move_model_request.name = obstacle_name
            # TODO 0.2 is the obstacle radius. it should be set automatically in future.
            move_model_request.pose.x, move_model_request.pose.y, move_model_request.pose.theta = get_random_pos_on_map(
                self._free_space_indices, self.map, 0.2, forbidden_zones)

            self._srv_move_model(move_model_request)

        for non_active_obstacle_name in non_active_obstacle_names:
            move_model_request = MoveModelRequest()
            move_model_request.name = non_active_obstacle_name
            move_model_request.pose = pos_non_active_obstacle
            self._srv_move_model(move_model_request)

    def generate_random_static_obstacle_yaml(self, folder_path: str, model_name: str = "random.model.yaml"):
        """generate a flatland model yaml file. The shape of the obstacle is randomly set.

        Args:
            model_name (str, optional): [description]. Defaults to "random.model.yaml".
        Return:
            obstacle_type
        """

        yaml_path = os.path.join(folder_path, model_name)
        # define body
        body = {}
        body["name"] = "random"
        body["pose"] = [0, 0, 0]
        body["type"] = "dynamic"
        body["color"] = [1, 0.2, 0.1, 1.0]  # [0.2, 0.8, 0.2, 0.75]
        body["footprints"] = []

        # define footprint
        f = {}
        footprints_shapes = ["circle", "polygon"]
        obstacle_type = random.choice(footprints_shapes)
        f["type"] = obstacle_type
        f["density"] = 0
        f["layers"] = ["all"]
        f["collision"] = 'true'
        f["sensor"] = "false"

        min_obstacle_radius = 0.5
        max_obstacle_radius = 1.5
        min_obstacle_vert = 3
        max_obstacle_vert = 6
        if f["type"] == "circle":
            f["radius"] = random.uniform(
                min_obstacle_radius, max_obstacle_radius)
        elif f["type"] == "polygon":
            f["points"] = []
            random_num_vert = random.randint(
                min_obstacle_vert, max_obstacle_vert)
            random_length = random.uniform(
                min_obstacle_radius, max_obstacle_radius)
            # radius_y=random.uniform(min_obstacle_radius,max_obstacle_radius)
            for i in range(random_num_vert):
                angle = 2 * math.pi * (float(i) / float(random_num_vert))
                vert = [math.cos(angle) * random_length,
                        math.sin(angle) * random_length]
                # print(vert)
                # print(angle)
                f["points"].append(vert)

        body["footprints"].append(f)
        # define dict_file
        dict_file = {'bodies': [body], "plugins": []}
        obstacle_type = 'static'
        # 0.5 possibility the obstacle is dynamic
        if random.randint(0, 1) == 1:
            # define plugin tween
            # tween modes. you can check https://flatland-simulator.readthedocs.io/en/latest/included_plugins/tween.html?highlight=tween for more details
            tween_modes = ['yoyo', 'once']
            x_range = (1, 5)
            y_range = x_range
            angle_range = (math.pi/4, -math.pi/4)
            x = random.uniform(x_range[0], x_range[1])
            y = random.uniform(y_range[0], y_range[1])
            angle = random.uniform(angle_range[0], angle_range[1])

            tween = {}
            tween['type'] = 'Tween'
            tween['name'] = 'Tween Plugin'
            tween['mode'] = random.choice(tween_modes)
            tween['delta'] = [x, y, angle]
            tween['duration'] = 10
            tween['body'] = 'random'
            dict_file['plugins'].append(tween)
            obstacle_type = 'dynamic'

        with open(yaml_path, 'w') as fd:
            yaml.dump(dict_file, fd)
        return obstacle_type

    def remove_obstacle(self, name: str):
        if len(self.obstacle_name_list) != 0:
            assert name in self.obstacle_name_list
        srv_request = DeleteModelRequest()
        srv_request.name = name
        response = self._srv_delete_model(srv_request)

        if not response.success:
            raise rospy.ServiceException(
                f"failed to remove the object with the name: {name}! ")
        else:
            rospy.logdebug(f"Removed the obstacle with the name {name}")

    def remove_obstacles(self, prefix_names: Union[list, None] = None):
        """remove all the obstacless belong to specific groups.
        Args:
            prefix_names (Union[list,None], optional): a list of group names. if it is None then all obstacles will
                be deleted. Defaults to None.
        """
        if len(self.obstacle_name_list) != 0:
            if prefix_names is None:
                group_names = '.'
            re_pattern = "^(?:" + '|'.join(prefix_names) + r')\w*'
            r = re.compile(re_pattern)
            to_be_removed_obstacles_names = list(
                filter(r.match, self.obstacle_name_list))
            for n in to_be_removed_obstacles_names:
                self.remove_obstacle(n)
        else:
            # it possible that in flatland there are still obstacles remaining when we create an instance of
            # this class.
            topics = rospy.get_published_topics()
            for t in topics:
                # the format of the topic is (topic_name,message_name)
                topic_name = t[0]
                object_name = topic_name.split("/")[-1]
                if object_name.startswith(self._obstacle_name_prefix):
                    self.remove_obstacle(object_name)
