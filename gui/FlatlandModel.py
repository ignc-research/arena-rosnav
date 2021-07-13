from os import path, truncate
from PyQt5 import QtGui, QtCore, QtWidgets
from enum import Enum
import yaml
import os
import numpy as np

class B2BodyType(Enum):
    DYNAMIC = 0
    STATIC = 1
    KINEMATIC = 2


class FlatlandFootprint():
    def __init__(self):
        self.layers = []
        self.collision = True
        self.density = 1.0

    def __eq__(self, other):
        if not isinstance(other, FlatlandFootprint):
            return NotImplemented

        return (self.layers == other.layers
                and self.collision == other.collision
                and np.allclose(self.density, other.density))

    @staticmethod
    def fromDict(d: dict):
        fp = FlatlandFootprint()
        # fill inherited class fields
        if d["type"] == "polygon":
            fp = PolygonFlatlandFootprint.fromDict(d)
        elif d["type"] == "circle":
            fp = CircleFlatlandFootprint.fromDict(d)
        else:
            raise Exception("unknown footprint type.")

        # fill base class fields
        if "layers" in d:
            fp.layers = d["layers"]
        if "collision" in d:
            fp.collision = d["collision"]
        if "density" in d:
            fp.density = float(d["density"])

        return fp

    def toDict(self):
        d = {}
        d["layers"] = self.layers
        d["collision"] = self.collision
        d["density"] = self.density
        return d

class CircleFlatlandFootprint(FlatlandFootprint):
    def __init__(self):
        super().__init__()
        self.center = [0.0, 0.0]
        self.radius = 0.5

    def __eq__(self, other):
        if not isinstance(other, CircleFlatlandFootprint):
            return NotImplemented

        return (super().__eq__(other)
                and np.allclose(self.center, other.center)
                and np.allclose(self.radius, other.radius))

    @staticmethod
    def fromDict(d: dict):
        fp = CircleFlatlandFootprint()
        if "center" in d:
            fp.center = [float(val) for val in d["center"]]
        if "radius" in d:
            fp.radius = float(d["radius"])
        return fp

    def toDict(self):
        d = super().toDict()
        d["center"] = self.center
        d["radius"] = self.radius
        d["type"] = "circle"
        return d

class PolygonFlatlandFootprint(FlatlandFootprint):
    def __init__(self):
        super().__init__()
        self.points = []

    def __eq__(self, other):
        if not isinstance(other, PolygonFlatlandFootprint):
            return NotImplemented

        if len(self.points) != len(other.points):
            return False

        return (super().__eq__(other)
                and np.allclose(self.points, other.points))

    @staticmethod
    def fromDict(d: dict):
        fp = PolygonFlatlandFootprint()
        if "points" in d:
            fp.points = [[float(point[0]), float(point[1])] for point in d["points"]]
        return fp

    def toDict(self):
        d = super().toDict()
        d["points"] = self.points
        d["type"] = "polygon"
        return d

class FlatlandBody():
    def __init__(self):
        self.name = "new_body"
        self.type = B2BodyType.DYNAMIC
        self.color = QtGui.QColor("red")
        self.linear_damping = 0.0
        self.angular_damping = 0.0
        self.footprints = []  # list of FlatlandFootprint objects

    def __eq__(self, other):
        if not isinstance(other, FlatlandBody):
            return NotImplemented

        # check everything else
        return (self.name == other.name
                and self.type == other.type
                and self.color == other.color
                and np.allclose(self.linear_damping, other.linear_damping)
                and np.allclose(self.angular_damping, other.angular_damping)
                and self.footprints == other.footprints)
        
    @staticmethod
    def fromDict(d: dict):
        body = FlatlandBody()
        if "name" in d:
            body.name = d["name"]
        if "type" in d:
            body.type = B2BodyType[d["type"].upper()]
        if "color" in d:
            rgba_values = [int(val * 255) for val in d["color"]]
            body.color = QtGui.QColor(rgba_values[0], rgba_values[1], rgba_values[2], rgba_values[3])
        if "linear_damping" in d:
            body.linear_damping = d["linear_damping"]
        if "angular_damping" in d:
            body.angular_damping = d["angular_damping"]
        if "footprints" in d:
            for footprint in d["footprints"]:
                body.footprints.append(FlatlandFootprint.fromDict(footprint))
        return body

    def toDict(self):
        '''
        Return this object as a dictionary.
        '''
        d = {}
        d["name"] = self.name
        d["color"] = [self.color.redF(), self.color.greenF(), self.color.blueF(), self.color.alphaF()]
        d["type"] = self.type.name.lower()
        d["linear_damping"] = self.linear_damping
        d["angular_damping"] = self.angular_damping
        d["footprints"] = [footprint.toDict() for footprint in self.footprints]
        return d

class FlatlandModel():
    def __init__(self):
        super().__init__()
        self.bodies = {}  # key: body id (int), value: body (FlatlandBody)
        self.path = ""  # path to file associated with this model
        self.bodies_index = 0

    def __eq__(self, other):
        if not isinstance(other, FlatlandModel):
            return NotImplemented

        if len(self.bodies.keys()) == len(other.bodies.keys()):
            for body1, body2 in zip(self.bodies.values(), other.bodies.values()):
                if body1 != body2:
                    return False
            return True

        return False

    def toDict(self):
        d = {}
        d["bodies"] = [body.toDict() for body in self.bodies.values()]
        return d

    def save(self, path_in = ""):
        if path_in == "" and self.path == "":
                return False
        elif path_in != "":
            self.path = path_in

        with open(self.path, "w") as file:
            data = self.toDict()
            yaml.dump(data, file, default_flow_style=None)

        print("saved model to", self.path)
        return True

    def load(self, path: str):
        if os.path.exists(path):
            self.bodies = {}
            with open(path, "r") as file:
                data = yaml.safe_load(file)
                for body in data["bodies"]:
                    flatland_body = FlatlandBody.fromDict(body)
                    self.bodies[self.bodies_index] = flatland_body
                    self.bodies_index += 1
            self.path = path


# Tests
def compare_test1():
    m1 = FlatlandModel()
    m1.bodies[0] = FlatlandBody()

    m2 = FlatlandModel()
    m2.bodies[0] = FlatlandBody()

    assert m1 == m2
    assert not m1 != m2
    print("compare_test1 passed")

def compare_test2():
    m1 = FlatlandModel()
    m1.bodies[0] = FlatlandBody()

    m2 = FlatlandModel()
    m2.bodies[1] = FlatlandBody()

    assert m1 != m2
    print("compare_test2 passed")

if __name__ == "__main__":
    compare_test1()
    compare_test2()
