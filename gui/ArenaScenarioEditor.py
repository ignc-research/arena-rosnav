from PyQt5.QtCore import QPoint, QPointF, Qt, pyqtSlot
from PyQt5.QtGui import QIcon, QPainterPath, QPixmap, QColor, QPolygonF, QTransform
from PyQt5.QtWidgets import QApplication, QGraphicsEllipseItem, QGraphicsItem, QGraphicsPixmapItem, QGraphicsTextItem, QLabel, QMainWindow, QMessageBox, QWidget, QGridLayout, QVBoxLayout, QFrame, QGraphicsScene, QGraphicsView, QScrollArea, QHBoxLayout, QPushButton, QSpacerItem, QSizePolicy, QGraphicsPathItem
import rospkg
import os
from typing import Tuple
from FlatlandBodyEditor import *
from PedsimAgentEditor import PedsimAgentEditor
from ArenaScenario import *
from QtExtensions import *

class RosMapData():
    def __init__(self, path: str = ""):
        self.image_path = ""
        self.resolution = 1.0
        self.origin = [0.0, 0.0, 0.0]
        self.path = path
        if path != "":
            self.load(path)

    def load(self, path: str):
        if os.path.exists(path):
            self.path = path
            with open(path, "r") as file:
                data = yaml.safe_load(file)
                folder_path = os.path.dirname(path)
                self.image_path = os.path.join(folder_path, data["image"])
                self.resolution = float(data["resolution"])
                self.origin = [float(value) for value in data["origin"]]



class WaypointWidget(QWidget):
    def __init__(self, pedsimAgentWidget, graphicsScene: QGraphicsScene, posIn: QPointF = None, **kwargs):
        super().__init__(**kwargs)
        self.id = 0
        self.pedsimAgentWidget = pedsimAgentWidget  # needed so the ellipse item can trigger a waypoint path redraw
        # create circle and add to scene
        self.ellipseItem = ArenaGraphicsEllipseItem(self, -0.25, -0.25, 0.5, 0.5)
        self.graphicsScene = graphicsScene
        graphicsScene.addItem(self.ellipseItem)
        # setup widgets
        self.setupUI()
        self.setId(self.id)
        # set initial position
        if posIn != None:
            self.setPos(posIn)

    def setupUI(self):
        self.setLayout(QtWidgets.QHBoxLayout())
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Maximum, QtWidgets.QSizePolicy.Policy.Maximum)

        # x value
        ## label
        label = QtWidgets.QLabel("x")
        self.layout().addWidget(label)
        ## spinbox
        self.posXSpinBox = ArenaQDoubleSpinBox()
        self.posXSpinBox.valueChanged.connect(self.updateEllipseItemFromSpinBoxes)
        self.layout().addWidget(self.posXSpinBox)

        # y value
        ## label
        label = QtWidgets.QLabel("y")
        self.layout().addWidget(label)
        ## spinbox
        self.posYSpinBox = ArenaQDoubleSpinBox()
        self.posYSpinBox.valueChanged.connect(self.updateEllipseItemFromSpinBoxes)
        self.layout().addWidget(self.posYSpinBox)

        # delete button
        delete_button = QtWidgets.QPushButton("X")
        delete_button.setFixedWidth(30)
        delete_button.setStyleSheet("background-color: red")
        delete_button.clicked.connect(self.remove)
        self.layout().addWidget(delete_button)

    def setId(self, id: int):
        self.id = id

    def setPos(self, posIn: QPointF):
        # set values of spin boxes (and ellipse item)
        # since spin boxes are connected to the ellipse item, the change will be propagated
        self.posXSpinBox.setValue(posIn.x())
        self.posYSpinBox.setValue(posIn.y())

    def getPos(self) -> Tuple[float, float]:
        return self.posXSpinBox.value(), self.posYSpinBox.value()

    def updateEllipseItemFromSpinBoxes(self):
        if not self.ellipseItem.isDragged:  # do this to prevent recursion between spin boxes and graphics item
            x = self.posXSpinBox.value()
            y = self.posYSpinBox.value()
            self.ellipseItem.setPosNoEvent(x, y)  # set without event to prevent recursion between spin boxes and graphics item

    def updateSpinBoxesFromGraphicsItem(self):
        new_pos = self.ellipseItem.mapToScene(self.ellipseItem.transformOriginPoint())
        self.posXSpinBox.setValue(new_pos.x())
        self.posYSpinBox.setValue(new_pos.y())

    def remove(self):
        self.ellipseItem.scene().removeItem(self.ellipseItem)
        del self.ellipseItem.keyPressEater  # delete to remove event filter

        self.parent().layout().removeWidget(self)
        self.pedsimAgentWidget.updateWaypointIdLabels()
        self.pedsimAgentWidget.drawWaypointPath()
        self.deleteLater()



class PedsimAgentWidget(QFrame):
    '''
    This is a row in the obstacles frame.
    '''
    def __init__(self, id: int, pedsimAgentIn: PedsimAgent, graphicsScene: QGraphicsScene, graphicsView: ArenaQGraphicsView, **kwargs):
        super().__init__(**kwargs)
        self.id = id
        self.graphicsScene = graphicsScene
        self.graphicsView = graphicsView
        self.pedsimAgent = pedsimAgentIn
        self.graphicsPathItem = None
        self.draggingItem = False
        self.initGraphicsPathItem()
        self.setup_ui()
        self.graphicsPathItem.updateTextItemText()
        self.pedsimAgentEditor = PedsimAgentEditor(self, parent=self.parent(), flags=QtCore.Qt.WindowType.Window)
        self.addWaypointModeActive = False
        self.activeModeWindow = ActiveModeWindow(self)
        graphicsView.clickedPos.connect(self.handleGraphicsViewClick)

        # GraphicsItem for drawing a path connecting the waypoints
        self.waypointPathItem = QGraphicsPathItem()
        ## create brush
        brush = QtGui.QBrush(QColor(), QtCore.Qt.BrushStyle.NoBrush)
        self.waypointPathItem.setBrush(brush)
        ## create pen
        pen = QtGui.QPen()
        pen.setColor(QColor("lightseagreen"))
        pen.setWidthF(0.1)
        pen.setStyle(QtCore.Qt.PenStyle.SolidLine)
        pen.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(QtCore.Qt.PenJoinStyle.RoundJoin)
        self.waypointPathItem.setPen(pen)
        ## add to scene
        graphicsScene.addItem(self.waypointPathItem)

        self.updateEverythingFromPedsimAgent()

    def setup_ui(self):
        self.setLayout(QtWidgets.QGridLayout())
        self.setFrameStyle(QtWidgets.QFrame.Shape.Box | QtWidgets.QFrame.Shadow.Raised)

        # name label
        self.name_label = QtWidgets.QLabel("Ped " + str(self.id))
        self.layout().addWidget(self.name_label, 0, 0)

        # edit button
        self.edit_button = QtWidgets.QPushButton("Edit")
        self.edit_button.clicked.connect(self.onEditClicked)
        self.layout().addWidget(self.edit_button, 0, 1)

        # delete button
        self.delete_button = QtWidgets.QPushButton("Delete")
        self.delete_button.clicked.connect(self.onDeleteClicked)
        self.layout().addWidget(self.delete_button, 0, 2)

        # position
        label = QtWidgets.QLabel("Pos:")
        self.layout().addWidget(label, 1, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        self.posXSpinBox = ArenaQDoubleSpinBox()
        self.posXSpinBox.valueChanged.connect(self.updateGraphicsPathItemFromSpinBoxes)
        self.layout().addWidget(self.posXSpinBox, 1, 1)
        self.posYSpinBox = ArenaQDoubleSpinBox()
        self.posYSpinBox.valueChanged.connect(self.updateGraphicsPathItemFromSpinBoxes)
        self.layout().addWidget(self.posYSpinBox, 1, 2)

        # waypoints
        label = QtWidgets.QLabel("Waypoints:")
        self.layout().addWidget(label, 2, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        button = QPushButton("Add Waypoints...")
        button.clicked.connect(self.onAddWaypointClicked)
        self.layout().addWidget(button, 2, 1, 1, -1)
        self.waypointListWidget = QWidget()
        self.waypointListWidget.setLayout(QVBoxLayout())
        self.layout().addWidget(self.waypointListWidget, 3, 0, 1, -1)

    def initGraphicsPathItem(self):
        self.graphicsPathItem = ArenaGraphicsPathItem(self)

        # create brush
        brush = QtGui.QBrush(QColor("red"), QtCore.Qt.BrushStyle.SolidPattern)
        self.graphicsPathItem.setBrush(brush)
        # create pen
        pen = QtGui.QPen()
        pen.setWidthF(0.01)
        pen.setStyle(QtCore.Qt.PenStyle.SolidLine)
        pen.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(QtCore.Qt.PenJoinStyle.RoundJoin)
        self.graphicsPathItem.setPen(pen)
        # add to scene
        self.graphicsScene.addItem(self.graphicsPathItem)

    def drawWaypointPath(self):
        path = QPainterPath()
        path.moveTo(self.getCurrentAgentPosition())
        for w in self.getWaypointWidgets():
            w_pos = w.ellipseItem.mapToScene(w.ellipseItem.transformOriginPoint())
            path.lineTo(w_pos)

        self.waypointPathItem.setPath(path)

    def getCurrentAgentPosition(self):
        x = self.posXSpinBox.value()
        y = self.posYSpinBox.value()
        return QPointF(x, y)

    def getWaypointWidgets(self):
        widgets = []
        for i in range(self.waypointListWidget.layout().count()):
            w = self.waypointListWidget.layout().itemAt(i).widget()
            if w != None:
                widgets.append(w)
        return widgets

    def updateSpinBoxesFromGraphicsItem(self):
        new_pos = self.graphicsPathItem.mapToScene(self.graphicsPathItem.transformOriginPoint())
        self.posXSpinBox.setValue(new_pos.x())
        self.posYSpinBox.setValue(new_pos.y())

    def updateGraphicsPathItemFromSpinBoxes(self):
        if not self.draggingItem:  # prevents recursive loop (spin box <-> moving item)
            x = self.posXSpinBox.value()
            y = self.posYSpinBox.value()
            self.graphicsPathItem.setPosNoEvent(x, y)
            self.graphicsPathItem.updateTextItemPos()

    def updateGraphicsPathItemFromPedsimAgent(self):
        painter_path = QPainterPath()
        painter_path.setFillRule(Qt.WindingFill)
        # get shapes from the agent and convert them to a path
        for body in self.pedsimAgent.flatlandModel.bodies.values():
            # skip safety distance circle
            if body.name == "safety_dist_circle":
                continue
            # set color
            brush = QtGui.QBrush(body.color, QtCore.Qt.BrushStyle.SolidPattern)
            self.graphicsPathItem.setBrush(brush)
            # compose path
            for footprint in body.footprints:
                if isinstance(footprint, CircleFlatlandFootprint):
                    center = QPointF(footprint.center[0], footprint.center[1])
                    radius = footprint.radius
                    painter_path.addEllipse(center, radius, radius)
                if isinstance(footprint, PolygonFlatlandFootprint):
                    polygon = QPolygonF([QPointF(point[0], point[1]) for point in footprint.points])
                    painter_path.addPolygon(polygon)
        self.graphicsPathItem.setPath(painter_path)

    def setPedsimAgent(self, agent: PedsimAgent):
        self.pedsimAgent = agent
        self.updateEverythingFromPedsimAgent()

    def updateEverythingFromPedsimAgent(self):
        # position
        self.posXSpinBox.setValue(self.pedsimAgent.pos.x)
        self.posYSpinBox.setValue(self.pedsimAgent.pos.y)
        # waypoints
        ## remove all waypoint widgets
        for w in self.getWaypointWidgets():
            w.remove()
        ## add new waypoints
        for wp in self.pedsimAgent.waypoints:
            pos = QPointF(wp.x, wp.y)
            self.addWaypoint(pos)
        # update name label
        self.name_label.setText(self.pedsimAgent.name)
        # update name in scene
        self.graphicsPathItem.updateTextItemText()
        # update item scene
        self.updateGraphicsPathItemFromPedsimAgent()

    def updateWaypointIdLabels(self):
        widgets = self.getWaypointWidgets()
        for i, w in enumerate(widgets):
            w.setId(i)

    @pyqtSlot(QPointF)
    def handleGraphicsViewClick(self, pos: QPointF):
        if self.addWaypointModeActive:
            self.addWaypoint(pos)

    def addWaypoint(self, pos: QPointF = None):
        w = WaypointWidget(self, self.graphicsScene, pos, parent=self)
        self.waypointListWidget.layout().addWidget(w)
        self.updateWaypointIdLabels()
        self.drawWaypointPath()

    def removeWaypoint(self, waypointWidget: WaypointWidget):
        self.waypointListWidget.layout().removeWidget(waypointWidget)
        self.updateWaypointIdLabels()
        self.drawWaypointPath()

    def setAddWaypointMode(self, enable: bool):
        self.addWaypointModeActive = enable
        if enable:
            self.activeModeWindow.show()
        else:
            self.activeModeWindow.hide()

    def save(self):
        # saves position and waypoints to the pedsim agent
        # all other attributes should have already been saved by the PedsimAgentEditor
        # position
        pos_x = self.posXSpinBox.value()
        pos_y = self.posYSpinBox.value()
        self.pedsimAgent.pos = Point(pos_x, pos_y, 0)
        # waypoints
        self.pedsimAgent.waypoints = []
        for w in self.getWaypointWidgets():
            x, y = w.getPos()
            self.pedsimAgent.waypoints.append(Point(x, y, 0))

    def remove(self):
        # remove waypoints
        for w in self.getWaypointWidgets():
            w.remove()
        # remove items from scene
        self.graphicsScene.removeItem(self.graphicsPathItem)
        self.graphicsScene.removeItem(self.graphicsPathItem.textItem)
        self.graphicsScene.removeItem(self.waypointPathItem)
        del self.graphicsPathItem.keyPressEater  # delete to remove event filter
        # remove widget
        self.parent().layout().removeWidget(self)
        self.deleteLater()

    def onAddWaypointClicked(self):
        if self.addWaypointModeActive:
            self.setAddWaypointMode(False)
        else:
            self.setAddWaypointMode(True)

    def onEditClicked(self):
        self.pedsimAgentEditor.show()

    def onDeleteClicked(self):
        self.remove()


class ArenaScenarioEditor(QMainWindow):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.setup_ui()
        self.arenaScenario = ArenaScenario()
        self.numObstacles = 0
        self.pixmap_item = None
        self.mapData = None
        self.currentSavePath = ""

    def setup_ui(self):
        self.setWindowTitle("Flatland Scenario Editor")
        self.resize(1300, 600)
        self.move(100, 100)
        icon = QIcon()
        icon.addPixmap(QPixmap('icon.png'), QIcon.Selected, QIcon.On)
        self.setWindowIcon(icon)

        # set central widget
        central_widget = QWidget()
        central_widget.setLayout(QGridLayout())
        self.setCentralWidget(central_widget)

        # menu bar
        menubar = self.menuBar()
        file_menu = menubar.addMenu("File")
        file_menu.addAction("New Scenario", self.onNewScenarioClicked, "Ctrl+N")
        file_menu.addAction("Open...", self.onOpenClicked, "Ctrl+O")
        file_menu.addAction("Save", self.onSaveClicked, "Ctrl+S")
        file_menu.addAction("Save As...", self.onSaveAsClicked, "Ctrl+Shift+S")
        add_menu = menubar.addMenu("Elements")
        add_menu.addAction("Set Map...", self.onSetMapClicked)
        add_menu.addAction("Add Pedsim Agent", self.onAddPedsimAgentClicked, "Ctrl+1")

        # drawing frame
        ## frame
        drawing_frame = QFrame()
        drawing_frame.setLayout(QVBoxLayout())
        drawing_frame.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.centralWidget().layout().addWidget(drawing_frame, 0, 1, -1, -1)
        ## graphicsscene
        self.gscene = QGraphicsScene()
        ## graphicsview
        self.gview = ArenaQGraphicsView(self.gscene)
        self.gview.scale(0.25, 0.25)  # zoom out a bit
        drawing_frame.layout().addWidget(self.gview)

        # obstacles
        ## scrollarea
        self.obstacles_scrollarea = QScrollArea(self)
        self.obstacles_scrollarea.setWidgetResizable(True)
        self.obstacles_scrollarea.setMinimumWidth(400)
        self.centralWidget().layout().addWidget(self.obstacles_scrollarea, 0, 0, -1, 1)
        ## frame
        self.obstacles_frame = QFrame()
        self.obstacles_frame.setLayout(QVBoxLayout())
        self.obstacles_frame.setSizePolicy(QtWidgets.QSizePolicy.Policy.Maximum, QtWidgets.QSizePolicy.Policy.Maximum)
        self.obstacles_scrollarea.setWidget(self.obstacles_frame)

    def onSetMapClicked(self):
        rospack = rospkg.RosPack()
        initial_folder = os.path.join(rospack.get_path("simulator_setup"), "maps")
        res = QtWidgets.QFileDialog.getOpenFileName(parent=self, directory=initial_folder)
        path = res[0]
        if path != "":
            self.setMap(path)

    def setMap(self, path: str):
        self.mapData = RosMapData(path)
        pixmap = QPixmap(self.mapData.image_path)
        transform = QTransform.fromScale(1.0, -1.0)  # flip y axis
        pixmap = pixmap.transformed(transform)
        if self.pixmap_item != None:
            # remove old map
            self.gscene.removeItem(self.pixmap_item)
        self.pixmap_item = QGraphicsPixmapItem(pixmap)
        self.pixmap_item.setZValue(-1.0)  # make sure map is always in the background
        self.pixmap_item.setScale(self.mapData.resolution)
        self.pixmap_item.setOffset(self.mapData.origin[0] / self.mapData.resolution, self.mapData.origin[1] / self.mapData.resolution)
        self.gscene.addItem(self.pixmap_item)

    def getMapData(self, path: str) -> dict:
        # read yaml file containing map meta data
        with open(path, "r") as file:
            data = yaml.safe_load(file)
            return data

    def disableAddWaypointMode(self):
        widgets = self.getPedsimAgentWidgets()
        for w in widgets:
            w.setAddWaypointMode(False)

    def onAddPedsimAgentClicked(self):
        rospack = rospkg.RosPack()
        yaml_file = ""
        try:
            yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
        except:
            pass

        new_agent = PedsimAgent("Ped " + str(self.numObstacles), yaml_file)
        self.arenaScenario.pedsimAgents.append(new_agent)
        self.addPedsimAgentWidget(new_agent)

    def addPedsimAgentWidget(self, agent: PedsimAgent) -> PedsimAgentWidget:
        '''
        Adds a new pedsim agent widget with the given agent.
        Warning: self.arenaScenario is not updated. Management of self.arenaScenario happens outside of this function.
        '''
        w = PedsimAgentWidget(self.numObstacles, agent, self.gscene, self.gview, parent=self)
        self.obstacles_frame.layout().addWidget(w)
        self.numObstacles += 1
        return w

    def getPedsimAgentWidgets(self):
        widgets = []
        for i in range(self.obstacles_frame.layout().count()):
            w = self.obstacles_frame.layout().itemAt(i).widget()
            if w != None and isinstance(w, PedsimAgentWidget):
                widgets.append(w)
        return widgets

    def keyPressEvent(self, event: QtGui.QKeyEvent):
        if event.key() == Qt.Key.Key_Escape or event.key() == Qt.Key.Key_Return:
            self.disableAddWaypointMode()
        return super().keyPressEvent(event)

    def onNewScenarioClicked(self):
        pass

    def onOpenClicked(self):
        rospack = rospkg.RosPack()
        initial_folder = os.path.join(rospack.get_path("simulator_setup"), "scenarios")
        res = QtWidgets.QFileDialog.getOpenFileName(parent=self, directory=initial_folder)
        path = res[0]
        if path != "":
            self.loadArenaScenario(path)

    def onSaveClicked(self):
        if not self.save():
            # no path has been set yet. fall back to "save as"
            self.onSaveAsClicked()

    def onSaveAsClicked(self) -> bool:
        rospack = rospkg.RosPack()
        initial_folder = os.path.join(rospack.get_path("simulator_setup"), "scenarios")

        res = QtWidgets.QFileDialog.getSaveFileName(parent=self, directory=initial_folder)
        path = res[0]
        if path != "":
            return self.save(path)

        return False

    def loadArenaScenario(self, path: str):
        self.currentSavePath = path
        self.arenaScenario.loadFromFile(path)
        self.updateWidgetsFromArenaScenario()

    def save(self, path: str = "") -> bool:
        if path != "":
            self.currentSavePath = path

        self.updateArenaScenarioFromWidgets()
        return self.arenaScenario.saveToFile()

    def updateWidgetsFromArenaScenario(self):
        # pedsim agents
        # remove all pedsim widgets
        for w in self.getPedsimAgentWidgets():
            w.remove()
        # create new pedsim widgets
        for agent in self.arenaScenario.pedsimAgents:
            self.addPedsimAgentWidget(agent)

        # interactive obstacles
        # TODO
        # robot position
        # TODO
        # robot goal
        # TODO

        # map
        self.setMap(self.arenaScenario.mapPath)

    def updateArenaScenarioFromWidgets(self):
        '''
        Save data from widgets into self.arenaScenario.
        '''
        # reset scenario
        self.arenaScenario.__init__()

        # save path
        self.arenaScenario.path = self.currentSavePath

        # save pedsim agents
        for w in self.getPedsimAgentWidgets():
            w.save()  # save all data from widget(s) into pedsim agent
            self.arenaScenario.pedsimAgents.append(w.pedsimAgent)  # add pedsim agent

        # save map path
        if self.mapData != None:
            self.arenaScenario.mapPath = self.mapData.path

        # TODO
        # robot position
        # robot goal
        # interactive obstacles



def normal_execution():
    app = QApplication([])

    widget = ArenaScenarioEditor()
    widget.show()

    app.exec()

def add_obstacle_test():
    app = QApplication([])

    widget = ArenaScenarioEditor()
    widget.onAddPedsimAgentClicked()
    widget.show()

    app.exec()

def test1():
    app = QApplication([])

    widget = ArenaScenarioEditor()
    widget.onAddPedsimAgentClicked()

    pedsim_widget = widget.getPedsimAgentWidgets()[0]
    pedsim_widget.pedsimAgentEditor.setModelPath("/home/daniel/catkin_ws/src/arena-rosnav/simulator_setup/dynamic_obstacles/big_forklift.model.yaml")
    pedsim_widget.pedsimAgentEditor.onSaveClicked()
    pedsim_widget.addWaypoint(QPointF(3, 4))
    widget.setMap("/home/daniel/catkin_ws/src/arena-rosnav/simulator_setup/maps/map1/map.yaml")
    

    widget.show()
    app.exec()

def edit_agent_test():
    app = QApplication([])

    widget = ArenaScenarioEditor()
    widget.setMap("/home/daniel/catkin_ws/src/arena-rosnav/simulator_setup/maps/map1/map.yaml")
    widget.onAddPedsimAgentClicked()

    pedsim_widget = widget.getPedsimAgentWidgets()[0]
    pedsim_widget.pedsimAgentEditor.setModelPath("/home/daniel/catkin_ws/src/arena-rosnav/simulator_setup/dynamic_obstacles/big_forklift.model.yaml")
    pedsim_widget.pedsimAgentEditor.onSaveClicked()
    pedsim_widget.addWaypoint(QPointF(3, 4))
    pedsim_widget.onEditClicked()
    

    widget.show()
    app.exec()

def save_test():
    app = QApplication([])
    widget = ArenaScenarioEditor()
    widget.onAddPedsimAgentClicked()
    widget.onAddPedsimAgentClicked()
    widget.setMap("/home/daniel/catkin_ws/src/arena-rosnav/simulator_setup/maps/map1/map.yaml")
    widget.currentSavePath = "/home/daniel/catkin_ws/src/arena-rosnav/simulator_setup/scenarios/test_scenario.json"
    # widget.arenaScenario.pedsimAgents.append(PedsimAgent())
    widget.onSaveClicked()

if __name__ == "__main__":
    normal_execution()
    # add_obstacle_test()
    # test1()
    # edit_agent_test()
    # save_test()
