from PyQt5 import QtGui, QtCore, QtWidgets
import numpy as np
import os
import yaml
import shutil
import pathlib
import re
import subprocess
from typing import List
from PIL import Image
from enum import Enum
from HelperFunctions import *
from QtExtensions import *


class MapType(Enum):
    INDOOR = 0
    OUTDOOR = 1


class MapGenerator(QtWidgets.QMainWindow):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # add graphicsscene and graphicsview
        self.scene = QtWidgets.QGraphicsScene()
        self.view = QtWidgets.QGraphicsView(self.scene)
        self.view.setDragMode(QtWidgets.QGraphicsView.DragMode.ScrollHandDrag)
        self.view.setSceneRect(-1000, -1000, 2000, 2000)
        self.view.fitInView(QtCore.QRectF(-1.5, -1.5, 100, 100),
                            mode=QtCore.Qt.AspectRatioMode.KeepAspectRatio)
        self.view.setHorizontalScrollBarPolicy(
            QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        self.setup_ui()
        self.updateWidgetsFromSelectedType(self.type_dropdown.currentIndex())
        self.showPreview()

    def setup_ui(self):
        self.setWindowTitle("Map Generator")
        self.resize(1100, 600)
        self.move(100, 100)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap('icon.png'),
                       QtGui.QIcon.Selected, QtGui.QIcon.On)
        self.setWindowIcon(icon)
        layout_index = 0

        # set central widget
        central_widget = QtWidgets.QWidget()
        central_widget.setLayout(QtWidgets.QGridLayout())
        self.setCentralWidget(central_widget)

        # splitter
        self.splitter = QtWidgets.QSplitter()
        self.centralWidget().layout().addWidget(self.splitter)

        # left side frame
        frame = QtWidgets.QFrame()
        frame.setFrameStyle(QtWidgets.QFrame.Shape.Box |
                            QtWidgets.QFrame.Shadow.Raised)
        frame.setLayout(QtWidgets.QGridLayout())
        frame.setSizePolicy(QtWidgets.QSizePolicy.Policy.Maximum,
                            QtWidgets.QSizePolicy.Policy.Maximum)
        self.splitter.addWidget(frame)

        # width
        # label
        width_label = QtWidgets.QLabel("### Width")
        width_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        frame.layout().addWidget(width_label, layout_index,
                                 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        # spinbox
        self.width_spin_box = QtWidgets.QSpinBox()
        self.width_spin_box.setRange(1, 10000)
        self.width_spin_box.setValue(101)
        self.width_spin_box.setSingleStep(1)
        self.width_spin_box.setFixedSize(150, 30)
        self.width_spin_box.valueChanged.connect(self.showPreview)
        frame.layout().addWidget(self.width_spin_box, layout_index,
                                 1, QtCore.Qt.AlignmentFlag.AlignRight)
        layout_index += 1

        # height
        # label
        height_label = QtWidgets.QLabel("### Height")
        height_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        frame.layout().addWidget(height_label, layout_index,
                                 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        # spinbox
        self.height_spin_box = QtWidgets.QSpinBox()
        self.height_spin_box.setRange(1, 10000)
        self.height_spin_box.setValue(101)
        self.height_spin_box.setSingleStep(1)
        self.height_spin_box.setFixedSize(150, 30)
        self.height_spin_box.valueChanged.connect(self.showPreview)
        frame.layout().addWidget(self.height_spin_box, layout_index,
                                 1, QtCore.Qt.AlignmentFlag.AlignRight)
        layout_index += 1

        # type
        # label
        type_label = QtWidgets.QLabel("### Type")
        type_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        frame.layout().addWidget(type_label, layout_index,
                                 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        # dropdown
        self.type_dropdown = QtWidgets.QComboBox()
        for map_type in MapType:
            self.type_dropdown.insertItem(
                map_type.value, map_type.name.lower())
        self.type_dropdown.setFixedSize(150, 30)
        self.type_dropdown.currentIndexChanged.connect(self.handleTypeChanged)
        frame.layout().addWidget(self.type_dropdown, layout_index,
                                 1, QtCore.Qt.AlignmentFlag.AlignRight)
        layout_index += 1

        # corridor width
        # label
        self.corridor_width_label = QtWidgets.QLabel("### Corridor Width")
        self.corridor_width_label.setTextFormat(
            QtCore.Qt.TextFormat.MarkdownText)
        frame.layout().addWidget(self.corridor_width_label,
                                 layout_index, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        # spinbox
        self.corridor_width_spin_box = QtWidgets.QSpinBox()
        self.corridor_width_spin_box.setRange(0, 1000)
        self.corridor_width_spin_box.setValue(3)
        self.corridor_width_spin_box.setSingleStep(1)
        self.corridor_width_spin_box.setFixedSize(150, 30)
        self.corridor_width_spin_box.valueChanged.connect(self.showPreview)
        frame.layout().addWidget(self.corridor_width_spin_box,
                                 layout_index, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        layout_index += 1

        # iterations
        # label
        self.iterations_label = QtWidgets.QLabel("### Iterations")
        self.iterations_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        frame.layout().addWidget(self.iterations_label, layout_index,
                                 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        # spinbox
        self.iterations_spin_box = QtWidgets.QSpinBox()
        self.iterations_spin_box.setRange(0, 1000)
        self.iterations_spin_box.setValue(100)
        self.iterations_spin_box.setSingleStep(1)
        self.iterations_spin_box.setFixedSize(150, 30)
        self.iterations_spin_box.valueChanged.connect(self.showPreview)
        frame.layout().addWidget(self.iterations_spin_box, layout_index,
                                 1, QtCore.Qt.AlignmentFlag.AlignRight)
        layout_index += 1

        # obstacles
        # label
        self.obstacles_label = QtWidgets.QLabel("### Obstacles")
        self.obstacles_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        frame.layout().addWidget(self.obstacles_label, layout_index,
                                 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        # spinbox
        self.obstacles_spin_box = QtWidgets.QSpinBox()
        self.obstacles_spin_box.setRange(0, 1000)
        self.obstacles_spin_box.setValue(20)
        self.obstacles_spin_box.setSingleStep(1)
        self.obstacles_spin_box.setFixedSize(150, 30)
        self.obstacles_spin_box.valueChanged.connect(self.showPreview)
        frame.layout().addWidget(self.obstacles_spin_box, layout_index,
                                 1, QtCore.Qt.AlignmentFlag.AlignRight)
        layout_index += 1

        # obstacle size
        # label
        self.obstacle_size_label = QtWidgets.QLabel("### Obstacle Size")
        self.obstacle_size_label.setTextFormat(
            QtCore.Qt.TextFormat.MarkdownText)
        frame.layout().addWidget(self.obstacle_size_label,
                                 layout_index, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        # spinbox
        self.obstacle_size_spin_box = QtWidgets.QSpinBox()
        self.obstacle_size_spin_box.setRange(0, 1000)
        self.obstacle_size_spin_box.setValue(2)
        self.obstacle_size_spin_box.setSingleStep(1)
        self.obstacle_size_spin_box.setFixedSize(150, 30)
        self.obstacle_size_spin_box.valueChanged.connect(self.showPreview)
        frame.layout().addWidget(self.obstacle_size_spin_box,
                                 layout_index, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        layout_index += 1

        # line
        line = Line()
        frame.layout().addWidget(line, layout_index, 0, 1, -1)
        layout_index += 1

        # number of maps
        # label
        number_of_maps_label = QtWidgets.QLabel("### Number of Maps")
        number_of_maps_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        frame.layout().addWidget(number_of_maps_label, layout_index,
                                 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        # spinbox
        self.number_of_maps_spin_box = QtWidgets.QSpinBox()
        self.number_of_maps_spin_box.setRange(0, 1000)
        self.number_of_maps_spin_box.setValue(5)
        self.number_of_maps_spin_box.setSingleStep(1)
        self.number_of_maps_spin_box.setFixedSize(150, 30)
        self.number_of_maps_spin_box.valueChanged.connect(
            self.numberOfMapsChanged)
        frame.layout().addWidget(self.number_of_maps_spin_box,
                                 layout_index, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        layout_index += 1

        # resolution
        # label
        resolution_label = QtWidgets.QLabel("### Map Resolution")
        resolution_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        frame.layout().addWidget(resolution_label, layout_index,
                                 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        # spinbox
        self.resolution_spin_box = QtWidgets.QDoubleSpinBox()
        self.resolution_spin_box.setRange(0, 1000)
        self.resolution_spin_box.setValue(0.5)
        self.resolution_spin_box.setSingleStep(0.01)
        self.resolution_spin_box.setFixedSize(150, 30)
        frame.layout().addWidget(self.resolution_spin_box, layout_index,
                                 1, QtCore.Qt.AlignmentFlag.AlignRight)
        layout_index += 1

        # folder
        folder_label = QtWidgets.QLabel("Save to Folder:")
        frame.layout().addWidget(folder_label, layout_index,
                                 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        browse_button = QtWidgets.QPushButton("Browse...")
        browse_button.clicked.connect(self.onBrowseClicked)
        frame.layout().addWidget(browse_button, layout_index,
                                 1, QtCore.Qt.AlignmentFlag.AlignRight)
        layout_index += 1
        self.folder_edit = QtWidgets.QLineEdit("Please select folder")
        # set default path to simulator_setup/maps if it exists
        sim_setup_path = get_ros_package_path("simulator_setup")
        if sim_setup_path != "":
            self.folder_edit.setText(os.path.join(sim_setup_path, "maps"))
        frame.layout().addWidget(self.folder_edit, layout_index, 0, 1, -1)
        layout_index += 1

        # generate maps button
        self.generate_maps_button = QtWidgets.QPushButton("Generate 5 maps")
        self.generate_maps_button.clicked.connect(self.onGenerateMapsClicked)
        frame.layout().addWidget(self.generate_maps_button, layout_index, 0, 1, -1)
        layout_index += 1

        # generate maps result label
        self.result_label = QtWidgets.QLabel("")
        self.result_label.setMaximumHeight(50)
        frame.layout().addWidget(self.result_label, layout_index, 0, 1, -1)
        layout_index += 1

        # generate maps result label animation
        self.result_label_animation = QtCore.QPropertyAnimation(
            self, b"text_color", self)
        self.result_label_animation.setDuration(250)
        self.result_label_animation.setLoopCount(2)
        self.result_label_animation.setStartValue(QtGui.QColor(255, 255, 255))
        self.result_label_animation.setEndValue(QtGui.QColor(0, 0, 0))

        # right side graphicsview
        self.splitter.addWidget(self.view)

        # set splitter sizes
        self.splitter.setSizes([300, 700])

    def handleTypeChanged(self):
        self.showPreview()
        self.updateWidgetsFromSelectedType(self.type_dropdown.currentIndex())

    def getTextColor(self) -> QtGui.QColor:
        return self.result_label.palette().text().color()

    def setTextColor(self, color: QtGui.QColor):
        palette = self.result_label.palette()
        palette.setColor(self.result_label.foregroundRole(), color)
        self.result_label.setPalette(palette)

    # text color property
    text_color = QtCore.pyqtProperty(QtGui.QColor, getTextColor, setTextColor)

    def numberOfMapsChanged(self, value):
        s = f"Generate {value} maps"
        self.generate_maps_button.setText(s)

    def getMapNames(self) -> List[str]:
        '''
        Generate simple map names that don't exist yet in the form of f"map{index}".
        Search the maps folder for already existing maps in this format. Get the highest index and then
        start counting from there.
        '''
        folder = pathlib.Path(self.folder_edit.text())
        map_folders = [p for p in folder.iterdir() if p.is_dir()]
        names = [p.parts[-1] for p in map_folders]
        # get only the names that are in the form of f"map{index}"
        prefix = "map"
        pat = re.compile(f"{prefix}\d+$", flags=re.ASCII)
        filtered_names = [name for name in names if pat.match(name) != None]
        # get the max index that already exists
        max_index = 0
        if len(filtered_names) > 0:
            max_index = max([int(name[len(prefix):])
                            for name in filtered_names])
        number_of_maps = self.number_of_maps_spin_box.value()
        # generate new names beginning with the max index
        return [f"map{i}" for i in range(max_index+1, max_index+1+number_of_maps)]

    def onGenerateMapsClicked(self):
        # generate maps
        height = self.height_spin_box.value()
        width = self.width_spin_box.value()
        path = pathlib.Path(self.folder_edit.text())

        # create new maps with appropriate names
        map_names = self.getMapNames()
        for map_name in map_names:
            map_array = self.getCurrentMap()
            if map_array is not None:
                self.make_image(map_array, path, map_name)
                self.create_yaml_files(path / map_name)

        # update result text
        if len(map_names) > 0:
            if len(map_names) == 1:
                self.result_label.setText(
                    f"Generated {len(map_names)} map: {map_names[0]}")
            elif len(map_names) > 1:
                self.result_label.setText(
                    f"Generated {len(map_names)} maps: {map_names[0]} - {map_names[-1]}")

            self.result_label_animation.start()

        # display maps in scene
        # remove old maps
        items = self.scene.items()
        for item in items:
            self.scene.removeItem(item)
        # add new maps
        offset_hor = 0
        offset_ver = 0
        for map_name in map_names:
            image_path = path / map_name / f"{map_name}.png"
            pixmap = QtGui.QPixmap(str(image_path))
            pixmap_item = QtWidgets.QGraphicsPixmapItem(pixmap)
            pixmap_item.setOffset(offset_hor, offset_ver)
            self.scene.addItem(pixmap_item)
            offset_ver += height + 10
        self.view.fitInView(QtCore.QRectF(-5, -5, width + 5, height +
                            (height / 3)), mode=QtCore.Qt.AspectRatioMode.KeepAspectRatio)

    def getCurrentMap(self) -> np.ndarray:
        map_type = MapType(self.type_dropdown.currentIndex())
        height = self.height_spin_box.value()
        width = self.width_spin_box.value()
        if height < 10 or width < 10:
            return None
        map_array = None
        if map_type == MapType.INDOOR:
            corridor_radius = self.corridor_width_spin_box.value()
            iterations = self.iterations_spin_box.value()
            map_array = self.create_indoor_map(
                height, width, corridor_radius, iterations)
        elif map_type == MapType.OUTDOOR:
            obstacle_number = self.obstacles_spin_box.value()
            obstacle_extra_radius = self.obstacle_size_spin_box.value()
            map_array = self.create_outdoor_map(
                height, width, obstacle_number, obstacle_extra_radius)

        return map_array

    def getXpmFromNdarray(self, a: np.ndarray) -> List[str]:
        height, width = a.shape
        xpm = []
        xpm.append(f"{width} {height} 2 1")
        xpm.append("1 c #000000")
        xpm.append("0 c #FFFFFF")
        for i in range(height):
            line = ""
            for j in range(width):
                line += str(int(a[i, j]))
            xpm.append(line)
        return xpm

    def showPreview(self):
        '''
        Generate an example map with the current settings and display it.
        '''
        # clear scene
        items = self.scene.items()
        for item in items:
            self.scene.removeItem(item)

        # generate a map
        map_array = self.getCurrentMap()

        if map_array is None:
            return

        # add map to the scene
        # generate XPM data from array
        xpm = self.getXpmFromNdarray(map_array)
        # get pixmap from XPM data
        pixmap = QtGui.QPixmap(xpm)
        pixmap_item = QtWidgets.QGraphicsPixmapItem(pixmap)
        # add to scene
        self.scene.addItem(pixmap_item)
        # adjust view
        height = self.height_spin_box.value()
        width = self.width_spin_box.value()
        self.view.fitInView(QtCore.QRectF(-5, -5, width + 5, height +
                            (height / 3)), mode=QtCore.Qt.AspectRatioMode.KeepAspectRatio)

    def onBrowseClicked(self):
        path = QtWidgets.QFileDialog.getExistingDirectory(
            self, "Select Maps Folder", str(pathlib.Path.home()))
        if os.path.exists(path):
            self.folder_edit.setText(path)

    def updateWidgetsFromSelectedType(self, index):
        map_type = MapType(index)
        if map_type == MapType.INDOOR:
            self.corridor_width_label.show()
            self.corridor_width_spin_box.show()
            self.iterations_label.show()
            self.iterations_spin_box.show()
            self.obstacles_label.hide()
            self.obstacles_spin_box.hide()
            self.obstacle_size_label.hide()
            self.obstacle_size_spin_box.hide()
        elif map_type == MapType.OUTDOOR:
            self.corridor_width_label.hide()
            self.corridor_width_spin_box.hide()
            self.iterations_label.hide()
            self.iterations_spin_box.hide()
            self.obstacles_label.show()
            self.obstacles_spin_box.show()
            self.obstacle_size_label.show()
            self.obstacle_size_spin_box.show()

    def create_yaml_files(self, map_folder_path: pathlib.Path):
        '''
        Create the files map.yaml (ROS) and map.wordl.yaml (Flatland) for the map.
        map_folder_path: path to folder for this map e.g.: /home/user/catkin_ws/src/arena-rosnav/simulator_setup/maps/mymap
        '''
        map_folder = pathlib.Path(map_folder_path)
        map_name = map_folder.parts[-1]

        # create map.yaml
        map_yaml = {
            "image": "{0}.png".format(map_name),
            "resolution": self.resolution_spin_box.value(),
            "origin": [0.0, 0.0, 0.0],  # [-x,-y,0.0]
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196
        }

        with open(str(map_folder / "map.yaml"), 'w') as outfile:
            yaml.dump(map_yaml, outfile, sort_keys=False,
                      default_flow_style=None)

        # create map.world.yaml
        world_yaml_properties = {
            "properties": {
                "velocity_iterations": 10,
                "position_iterations": 10
            }
        }

        world_yaml_layers = {
            "layers": [
                {
                    "name": "static",
                    "map": "map.yaml",
                    "color": [0, 1, 0, 1]
                }
            ]
        }

        with open(str(map_folder / "map.world.yaml"), 'w') as outfile:
            # somehow the first part must be with default_flow_style=False
            yaml.dump(world_yaml_properties, outfile,
                      sort_keys=False, default_flow_style=False)
            # 2nd part must be with default_flow_style=None
            yaml.dump(world_yaml_layers, outfile,
                      sort_keys=False, default_flow_style=None)

    def make_image(self, map: np.ndarray, maps_folder_path: pathlib.Path, map_name: str):
        '''
        Create PNG file from occupancy map (1:occupied, 0:free) and the necessary yaml files.
        - map: numpy array
        - maps_folder_path: path to maps folder e.g.: /home/user/catkin_ws/src/arena-rosnav/simulator_setup/maps
        - map_name: name of map, a folder will be created using this name
        '''
        # create new directory for map
        map_folder = maps_folder_path / map_name
        if not map_folder.exists():
            os.mkdir(str(map_folder))
        # create image
        # monochromatic image
        img = Image.fromarray(((map-1)**2*255).astype('uint8'))
        imgrgb = img.convert('RGB')
        # save image
        # save map in map directory
        imgrgb.save(str(map_folder / (map_name + ".png")))

    # create empty map with format given by height,width and initialize empty tree
    def initialize_map(self, height, width, type="indoor"):
        if type == "outdoor":
            map = np.tile(1, [height, width])
            map[slice(1, height-1), slice(1, width-1)] = 0
            return map
        else:
            return np.tile(1, [height, width])

    def insert_root_node(self, map, tree):  # create root node in center of map
        root_node = [int(np.floor(map.shape[0]/2)),
                     int(np.floor(map.shape[1]/2))]
        map[root_node[0], root_node[1]] = 0
        tree.append(root_node)

    # sample position from map within boundary and leave tolerance for corridor width
    def sample(self, map, corridor_radius):
        random_x = np.random.choice(
            range(corridor_radius+2, map.shape[0]-corridor_radius-1, 1))
        random_y = np.random.choice(
            range(corridor_radius+2, map.shape[1]-corridor_radius-1, 1))
        return [random_x, random_y]

    # find nearest node according to L1 norm
    def find_nearest_node(self, random_position, tree):
        nearest_node = []
        min_distance = np.inf
        for node in tree:
            distance = sum(np.abs(np.array(random_position)-np.array(node)))
            if distance < min_distance:
                min_distance = distance
                nearest_node = node
        return nearest_node

    # insert new node into the map and tree
    def insert_new_node(self, random_position, tree, map):
        map[random_position[0], random_position[1]] = 0
        tree.append(random_position)

    def get_constellation(self, node1, node2):
        # there are two relevant constellations for the 2 nodes, which must be considered when creating the horizontal and vertical path
        # 1: lower left and upper right
        # 2: upper left and lower right
        # each of the 2 constellation have 2 permutations which must be considered as well
        constellation1 = {
            # x1>x2 and y1<y2
            "permutation1": node1[0] > node2[0] and node1[1] < node2[1],
            "permutation2": node1[0] < node2[0] and node1[1] > node2[1]}  # x1<x2 and y1>y2
        if constellation1["permutation1"] or constellation1["permutation2"]:
            return 1
        else:
            return 2

    def create_path(self, node1, node2, corridor_radius, map):
        coin_flip = np.random.random()
        # x and y coordinates must be sorted for usage with range function
        x1, x2 = sorted([node1[0], node2[0]])
        y1, y2 = sorted([node1[1], node2[1]])
        if self.get_constellation(node1, node2) == 1:  # check which constellation
            # randomly determine the curvature of the path (right turn/left turn)
            if coin_flip >= 0.5:
                map[slice(x1-corridor_radius, x1+corridor_radius+1), range(y1 -
                                                                           corridor_radius, y2+1+corridor_radius, 1)] = 0  # horizontal path
                map[range(x1-corridor_radius, x2+1+corridor_radius, 1), slice(y1 -
                                                                              corridor_radius, y1+corridor_radius+1)] = 0  # vertical path
            else:
                map[slice(x2-corridor_radius, x2+corridor_radius+1), range(y1 -
                                                                           corridor_radius, y2+1+corridor_radius, 1)] = 0  # horizontal path
                map[range(x1-corridor_radius, x2+1+corridor_radius, 1), slice(y2 -
                                                                              corridor_radius, y2+corridor_radius+1)] = 0  # vertical path
        else:
            # randomly determine the curvature of the path (right turn/left turn)
            if coin_flip >= 0.5:
                map[slice(x1-corridor_radius, x1+corridor_radius+1), range(y1 -
                                                                           corridor_radius, y2+1+corridor_radius, 1)] = 0  # horizontal path
                map[range(x1-corridor_radius, x2+1+corridor_radius, 1), slice(y2 -
                                                                              corridor_radius, y2+corridor_radius+1)] = 0  # vertical path
            else:
                map[slice(x2-corridor_radius, x2+corridor_radius+1), range(y1 -
                                                                           corridor_radius, y2+1+corridor_radius, 1)] = 0  # horizontal path
                map[range(x1-corridor_radius, x2+1+corridor_radius, 1), slice(y1 -
                                                                              corridor_radius, y1+corridor_radius+1)] = 0  # vertical path

    def create_indoor_map(self, height, width, corridor_radius, iterations):
        tree = []  # initialize empty tree
        map = self.initialize_map(height, width)
        self.insert_root_node(map, tree)
        for i in range(iterations):  # create as many paths/nodes as defined in iteration
            random_position = self.sample(map, corridor_radius)
            # nearest node must be found before inserting the new node into the tree, else nearest node will be itself
            nearest_node = self.find_nearest_node(random_position, tree)
            self.insert_new_node(random_position, tree, map)
            self.create_path(random_position, nearest_node,
                             corridor_radius, map)
        return map

    def create_outdoor_map(self, height, width, obstacle_number, obstacle_extra_radius):
        map = self.initialize_map(height, width, type="outdoor")
        for i in range(obstacle_number):
            random_position = self.sample(map, obstacle_extra_radius)
            map[slice(random_position[0]-obstacle_extra_radius, random_position[0]+obstacle_extra_radius+1),  # create 1 pixel obstacles with extra radius if specified
                slice(random_position[1]-obstacle_extra_radius, random_position[1]+obstacle_extra_radius+1)] = 1
        return map


if __name__ == "__main__":
    app = QtWidgets.QApplication([])

    widget = MapGenerator()
    widget.show()

    app.exec()
