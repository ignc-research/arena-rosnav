from FlatlandModel import FlatlandModel
import numpy as np
from enum import Enum
from pedsim_msgs.msg import Ped, InteractiveObstacle
from geometry_msgs.msg import Point
import rospkg
import os
import yaml
import json

class PedsimStartupMode(Enum):
    DEFAULT = 0
    WAIT_TIMER = 1
    TRIGGER_ZONE = 2

class PedsimWaypointMode(Enum):
    LOOP = 0
    RANDOM = 1

class PedsimAgentType(Enum):
    ADULT = 0
    CHILD = 1
    ELDER = 2
    VEHICLE = 3
    SERVICEROBOT = 4

class InteractiveObstacleType(Enum):
    SHELF = 0

# class InteractiveObstacle():
#     def __init__(self) -> None:
#         self.obstacleType = InteractiveObstacleType.SHELF
#           TODO...

class PedsimAgent(Ped):
    def __init__(self, name = "", flatlandModelPath = "") -> None:
        super().__init__()
        self.name = name
        self.flatlandModel = None  # FlatlandModel instance
        if flatlandModelPath != "":
            self.loadFlatlandModel(flatlandModelPath)

        # set some default values
        self.type = "adult"

        self.number_of_peds = 1
        self.vmax = 1.0

        self.start_up_mode = "default"
        self.wait_time = 0
        self.trigger_zone_radius = 0

        self.chatting_probability = 0.01
        self.tell_story_probability = 0
        self.group_talking_probability = 0.01
        self.talking_and_walking_probability = 0.01
        self.requesting_service_probability = 0.01
        self.requesting_guide_probability = 0.01
        self.requesting_follower_probability = 0.01

        self.talking_base_time = 10.0
        self.tell_story_base_time = 0
        self.group_talking_base_time = 10.0
        self.talking_and_walking_base_time = 6.0
        self.receiving_service_base_time = 20.0
        self.requesting_service_base_time = 30.0

        self.max_talking_distance = 5.0
        self.max_servicing_radius = 5.0

        self.force_factor_desired = 1.0
        self.force_factor_obstacle = 1.0
        self.force_factor_social = 5.0
        self.force_factor_robot = 0.0

        self.waypoint_mode = 0

    def loadFlatlandModel(self, path: str):
        self.yaml_file = path
        model = FlatlandModel()
        model.load(path)
        self.flatlandModel = model

    def __eq__(self, other):
        if not isinstance(other, PedsimAgent):
            return NotImplemented

        if self.name != other.name:
            return False
        if self.flatlandModel != other.flatlandModel:
            return False

        if self.id != other.id:
            return False
        if not np.allclose([self.pos.x, self.pos.y, self.pos.z], [other.pos.x, other.pos.y, other.pos.z]):
            return False
        if self.type != other.type:
            return False
        if self.yaml_file != other.yaml_file:
            return False
        if self.number_of_peds != other.number_of_peds:
            return False
        if not np.allclose(self.vmax, other.vmax):
            return False
        if self.start_up_mode != other.start_up_mode:
            return False
        if not np.allclose(self.wait_time, other.wait_time):
            return False
        if not np.allclose(self.trigger_zone_radius, other.trigger_zone_radius):
            return False
        if not np.allclose(self.chatting_probability, other.chatting_probability):
            return False
        if not np.allclose(self.tell_story_probability, other.tell_story_probability):
            return False
        if not np.allclose(self.group_talking_probability, other.group_talking_probability):
            return False
        if not np.allclose(self.talking_and_walking_probability, other.talking_and_walking_probability):
            return False
        if not np.allclose(self.requesting_service_probability, other.requesting_service_probability):
            return False
        if not np.allclose(self.requesting_guide_probability, other.requesting_guide_probability):
            return False
        if not np.allclose(self.requesting_follower_probability, other.requesting_follower_probability):
            return False
        if not np.allclose(self.max_talking_distance, other.max_talking_distance):
            return False
        if not np.allclose(self.max_servicing_radius, other.max_servicing_radius):
            return False
        if not np.allclose(self.talking_base_time, other.talking_base_time):
            return False
        if not np.allclose(self.tell_story_base_time, other.tell_story_base_time):
            return False
        if not np.allclose(self.group_talking_base_time, other.group_talking_base_time):
            return False
        if not np.allclose(self.talking_and_walking_base_time, other.talking_and_walking_base_time):
            return False
        if not np.allclose(self.receiving_service_base_time, other.receiving_service_base_time):
            return False
        if not np.allclose(self.requesting_service_base_time, other.requesting_service_base_time):
            return False
        if not np.allclose(self.force_factor_desired, other.force_factor_desired):
            return False
        if not np.allclose(self.force_factor_obstacle, other.force_factor_obstacle):
            return False
        if not np.allclose(self.force_factor_social, other.force_factor_social):
            return False
        if not np.allclose(self.force_factor_robot, other.force_factor_robot):
            return False
        if len(self.waypoints) != len(other.waypoints):
            return False
        if not np.all([np.allclose([wpa.x, wpa.y, wpa.z], [wpb.x, wpb.y, wpb.z]) for wpa, wpb in zip(self.waypoints, other.waypoints)]):
            return False
        if self.waypoint_mode != other.waypoint_mode:
            return False

        return True

    def toDict(self):
        d = {}

        d["name"] = self.name
        
        d["id"] = self.id
        d["pos"] = [self.pos.x, self.pos.y, self.pos.z]
        d["type"] = self.type
        d["yaml_file"] = self.yaml_file
        d["number_of_peds"] = self.number_of_peds
        d["vmax"] = self.vmax

        d["start_up_mode"] = self.start_up_mode
        d["wait_time"] = self.wait_time
        d["trigger_zone_radius"] = self.trigger_zone_radius

        d["chatting_probability"] = self.chatting_probability
        d["tell_story_probability"] = self.tell_story_probability
        d["group_talking_probability"] = self.group_talking_probability
        d["talking_and_walking_probability"] = self.talking_and_walking_probability
        d["requesting_service_probability"] = self.requesting_service_probability
        d["requesting_guide_probability"] = self.requesting_guide_probability
        d["requesting_follower_probability"] = self.requesting_follower_probability

        d["max_talking_distance"] = self.max_talking_distance
        d["max_servicing_radius"] = self.max_servicing_radius

        d["talking_base_time"] = self.talking_base_time
        d["tell_story_base_time"] = self.tell_story_base_time
        d["group_talking_base_time"] = self.group_talking_base_time
        d["talking_and_walking_base_time"] = self.talking_and_walking_base_time
        d["receiving_service_base_time"] = self.receiving_service_base_time
        d["requesting_service_base_time"] = self.requesting_service_base_time

        d["force_factor_desired"] = self.force_factor_desired
        d["force_factor_obstacle"] = self.force_factor_obstacle
        d["force_factor_social"] = self.force_factor_social
        d["force_factor_robot"] = self.force_factor_robot

        d["waypoints"] = [[wp.x, wp.y, wp.z] for wp in self.waypoints]
        d["waypoint_mode"] = self.waypoint_mode

        return d

    @staticmethod
    def fromDict(d : dict):
        a = PedsimAgent(d["name"], d["yaml_file"])

        a.name = d["name"]
    
        a.id = d["id"]
        a.pos = Point(d["pos"][0], d["pos"][1], d["pos"][2])
        a.type = d["type"]
        a.yaml_file = d["yaml_file"]
        a.number_of_peds = d["number_of_peds"]
        a.vmax = d["vmax"]

        a.start_up_mode = d["start_up_mode"]
        a.wait_time = d["wait_time"]
        a.trigger_zone_radius = d["trigger_zone_radius"]

        a.chatting_probability = d["chatting_probability"]
        a.tell_story_probability = d["tell_story_probability"]
        a.group_talking_probability = d["group_talking_probability"]
        a.talking_and_walking_probability = d["talking_and_walking_probability"]
        a.requesting_service_probability = d["requesting_service_probability"]
        a.requesting_guide_probability = d["requesting_guide_probability"]
        a.requesting_follower_probability = d["requesting_follower_probability"]

        a.max_talking_distance = d["max_talking_distance"]
        a.max_servicing_radius = d["max_servicing_radius"]

        a.talking_base_time = d["talking_base_time"]
        a.tell_story_base_time = d["tell_story_base_time"]
        a.group_talking_base_time = d["group_talking_base_time"]
        a.talking_and_walking_base_time = d["talking_and_walking_base_time"]
        a.receiving_service_base_time = d["receiving_service_base_time"]
        a.requesting_service_base_time = d["requesting_service_base_time"]

        a.force_factor_desired = d["force_factor_desired"]
        a.force_factor_obstacle = d["force_factor_obstacle"]
        a.force_factor_social = d["force_factor_social"]
        a.force_factor_robot = d["force_factor_robot"]

        a.waypoints = [Point(wp[0], wp[1], wp[2]) for wp in d["waypoints"]]
        a.waypoint_mode = d["waypoint_mode"]

        return a

class ArenaScenario():
    def __init__(self):
        self.pedsimAgents = []  # list of PedsimAgent objects
        self.interactiveObstacles = []  # list of InteractiveObstacle messages
        self.robotPosition = np.zeros(2)  # starting position of robot
        self.robotGoal = np.zeros(2)  # robot goal
        self.mapPath = ""  # path to map file

        self.path = ""  # path to file associated with this scenario

    def toDict(self):
        d = {}

        d["pedsim_agents"] = [a.toDict() for a in self.pedsimAgents]
        # d["interactive_obstacles"] = TODO...
        d["robot_position"] = [float(value) for value in self.robotPosition]
        d["robot_goal"] = [float(value) for value in self.robotGoal]
        d["map_path"] = self.mapPath

        return d

    @staticmethod
    def fromDict(d : dict):
        scenario = ArenaScenario()
        scenario.loadFromDict(d)
        return scenario

    def loadFromDict(self, d: dict):
        # # reset values
        # self.__init__()

        self.pedsimAgents = [PedsimAgent.fromDict(a) for a in d["pedsim_agents"]]
        # self.interactiveObstacles = ...TODO
        self.robotPosition = np.array([d["robot_position"][0], d["robot_position"][1]])
        self.robotGoal = np.array([d["robot_goal"][0], d["robot_goal"][1]])
        self.mapPath = d["map_path"]

    def saveToFile(self, path_in: str = "", format: str = "json") -> bool:
        '''
        Save Scenario in file.
        - path_in: path to save file
        - format: format of save file, can be "json" or "yaml"
        '''
        if os.path.exists(path_in):
            self.path = path_in

        if self.path == "":
            return False

        with open(self.path, "w") as file:
            data = self.toDict()
            if format == "json":
                json.dump(data, file, indent=4)
                # json.dump(data, file, sort_keys=True, indent=4)
            elif format == "yaml":
                yaml.dump(data, file, default_flow_style=None)
            else:
                raise Exception(f"given file format '{format}' is not valid, needs to be 'json' or 'yaml'")

        return True

    def loadFromFile(self, path_in: str):
        if os.path.exists(path_in):
            _, file_extension = os.path.splitext(path_in)
            with open(path_in, "r") as f:
                data = None
                if file_extension == ".json":
                    data = json.load(f)
                elif file_extension == ".yaml":
                    data = yaml.safe_load(f)
                else:
                    raise Exception("wrong format. file needs to have 'json' or 'yaml' file ending.")

                self.loadFromDict(data)
                self.path = path_in

        else:
            raise Exception(f"file '{path_in}' does not exist")
