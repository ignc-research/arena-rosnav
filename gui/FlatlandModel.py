from PyQt5 import QtGui, QtCore, QtWidgets
from enum import Enum

class B2BodyType(Enum):
    DYNAMIC = 0
    STATIC = 1
    KINEMATIC = 2


class FlatlandFootprint():
    def __init__(self):
        self.layers = []
        self.collision = True
        self.density = 1.0


class CircleFlatlandFootprint(FlatlandFootprint):
    def __init__(self):
        super().__init__()
        self.center = [0.0, 0.0]
        self.radius = 0.5


class PolygonFlatlandFootprint(FlatlandFootprint):
    def __init__(self):
        super().__init__()
        self.points = []


class FlatlandBody():
    def __init__(self):
        self.name = "new_body"
        self.type = B2BodyType.DYNAMIC
        self.color = QtGui.QColor("red")
        self.linear_damping = 0.0
        self.angular_damping = 0.0
        self.footprints = []  # list of FlatlandFootprint objects


class FlatlandModel(QtCore.QObject):
    def __init__(self):
        super().__init__()
        self.bodies = {}  # key: body name (string), value: body (FlatlandBody)
