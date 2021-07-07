from FlatlandModel import FlatlandModel
import numpy as np
from enum import Enum
from pedsim_msgs.msg import Ped, InteractiveObstacle

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

class PedsimAgent(Ped):
    def __init__(self, name = "", flatlandModelPath = "") -> None:
        super().__init__()
        self.name = name
        self.flatlandModel = None  # FlatlandModel
        if flatlandModelPath != "":
            self.loadFlatlandModel(flatlandModelPath)
#         self.position = np.zeros(2)
#         self.agentType = PedsimAgentType.ADULT
#         self.vMax = 1.0
#         self.chattingProbability = 0.01
#         self.waypoints = []  # list of 2D waypoints
#         self.waypointMode = PedsimWaypointMode.LOOP
#         # TODO rest of attributes...

    def loadFlatlandModel(self, path: str):
        self.yaml_file = path
        model = FlatlandModel()
        model.load(path)
        self.flatlandModel = model

class ArenaScenario():
    def __init__(self) -> None:
        self.pedsimAgents = []  # list of PedsimAgent objects
        self.interactiveObstacles = []  # list of InteractiveObstacle messages
        self.robotPosition = np.zeros(2)  # starting position of robot
        self.robotGoal = np.zeros(2)  # robot goal
        self.mapPath = ""  # path to map file