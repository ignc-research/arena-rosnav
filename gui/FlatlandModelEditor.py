from PyQt5 import QtGui, QtCore, QtWidgets
import rospkg
import os
from FlatlandModel import *
from FlatlandBodyEditor import FlatlandBodyEditor

class FlatlandBodyWidget(QtWidgets.QWidget):
    '''
    This is a row in the bodies frame. It has the following elements:
        - body name label
        - edit button
        - delete button
    '''
    def __init__(self, id: int, flatland_model: FlatlandModel, create_new_flatland_body = True):
        super().__init__()
        self.setup_ui()
        self.id = id
        self.model = flatland_model
        if create_new_flatland_body:
            flatland_body = FlatlandBody()
            self.model.bodies[id] = flatland_body

        self.flatland_body_editor = FlatlandBodyEditor(id, self.model.bodies[id], parent=self, flags=QtCore.Qt.WindowType.Window)

    def setup_ui(self):
        self.layout = QtWidgets.QHBoxLayout()
        self.setLayout(self.layout)

        # add name label
        self.name_label = QtWidgets.QLabel("new_body")
        self.layout.addWidget(self.name_label)

        # add edit button
        self.edit_button = QtWidgets.QPushButton("edit")
        self.edit_button.clicked.connect(self.on_edit_clicked)
        self.layout.addWidget(self.edit_button)

        # add delete button
        self.delete_button = QtWidgets.QPushButton("delete")
        self.delete_button.clicked.connect(self.on_delete_clicked)
        self.layout.addWidget(self.delete_button)

    def on_edit_clicked(self):
        self.flatland_body_editor.show()

    def on_delete_clicked(self):
        self.model.bodies.pop(self.id)
        self.parent().layout().removeWidget(self)
        self.deleteLater()

class FlatlandModelEditor(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setup_ui()
        self.model = FlatlandModel()
        self.body_id = 0

    def setup_ui(self):
        self.setWindowTitle("Flatland Model Editor")
        central_widget = QtWidgets.QWidget()
        central_widget.setLayout(QtWidgets.QVBoxLayout())
        self.setCentralWidget(central_widget)
        menubar = self.menuBar()
        file_menu = menubar.addMenu("File")
        file_menu.addAction("Open", self.on_open_clicked, "Ctrl+O")
        file_menu.addAction("Save", self.on_save_clicked, "Ctrl+S")
        file_menu.addAction("Save As...", self.on_save_as_clicked, "Ctrl+Shift+S")

        # add frame to add and edit bodies
        self.setup_bodies_frame()

        # add expanding spacer item
        spacer = QtWidgets.QSpacerItem(1, 1, vPolicy=QtWidgets.QSizePolicy.Policy.Expanding)
        self.centralWidget().layout().addSpacerItem(spacer)

        self.resize(500, 200)

    def setup_bodies_frame(self):
        frame = QtWidgets.QFrame()
        frame.setLayout(QtWidgets.QGridLayout())

        # add title
        bodies_label = QtWidgets.QLabel("## Bodies")
        bodies_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        frame.layout().addWidget(bodies_label, 0, 0, QtCore.Qt.AlignmentFlag.AlignLeft)

        # add "add body" button
        self.add_body_button = QtWidgets.QPushButton("Add Body")
        self.add_body_button.setFixedSize(200, 30)
        self.add_body_button.clicked.connect(self.on_add_body_button_clicked)
        frame.layout().addWidget(self.add_body_button, 0, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # add body list
        self.bodies_list_frame = QtWidgets.QFrame()
        self.bodies_list_frame.setLayout(QtWidgets.QVBoxLayout())
        frame.layout().addWidget(self.bodies_list_frame, 1, 0, 1, -1)

        # add this frame to layout
        self.centralWidget().layout().addWidget(frame)

    def get_body_widgets(self):
        widgets = []
        for i in range(self.bodies_list_frame.layout().count()):
            w = self.bodies_list_frame.layout().itemAt(i).widget()
            if w != None:
                widgets.append(w)
        return widgets

    def remove_all_bodies(self):
        widgets = self.get_body_widgets()
        for widget in widgets:
            widget.on_delete_clicked()

    def on_add_body_button_clicked(self):
        w = FlatlandBodyWidget(self.body_id, self.model)
        return self.add_flatland_body_widget(w)

    def add_flatland_body_widget(self, w: FlatlandBodyWidget):
        self.bodies_list_frame.layout().addWidget(w)
        self.body_id += 1
        return w

    def on_open_clicked(self, path_in=""):
        path = ""
        if path_in == "":
            rospack = rospkg.RosPack()
            initial_folder = os.path.join(rospack.get_path("simulator_setup"), "obstacles")
            res = QtWidgets.QFileDialog.getOpenFileName(parent=self, directory=initial_folder)
            path = res[0]
        else:
            path = path_in
        
        if path != "":
            self.setWindowTitle(path.split("/")[-1])
            self.remove_all_bodies()
            self.model.load(path)
            for body in self.model.bodies.values():
                w = FlatlandBodyWidget(self.body_id, self.model, create_new_flatland_body=False)
                self.add_flatland_body_widget(w)

                # w = self.on_add_body_button_clicked()
                # w.flatland_body_editor.set_flatland_body(body)

    def on_save_clicked(self):
        res = self.model.save()
        if not res:
            QtWidgets.QMessageBox.information(self, "Error", "No file path set for the current model. Can't save.")

    def on_save_as_clicked(self):
        rospack = rospkg.RosPack()
        initial_folder = os.path.join(rospack.get_path("simulator_setup"), "obstacles")

        res = QtWidgets.QFileDialog.getSaveFileName(parent=self, directory=initial_folder)
        path = res[0]
        if path != "":
            self.model.save(path)

def normal_execution():
    app = QtWidgets.QApplication([])

    widget = FlatlandModelEditor()
    widget.show()

    app.exec()

def test_save():
    app = QtWidgets.QApplication([])

    widget = FlatlandModelEditor()
    w = widget.on_add_body_button_clicked()
    w.flatland_body_editor.on_add_polygon_clicked()
    w.flatland_body_editor.on_save_clicked()
    widget.model.save("/home/daniel/catkin_ws/src/arena-rosnav/simulator_setup/obstacles/test.yaml")

def test_open():
    app = QtWidgets.QApplication([])

    widget = FlatlandModelEditor()
    widget.on_open_clicked(path_in="/home/daniel/catkin_ws/src/arena-rosnav/simulator_setup/obstacles/cleaner.model.yaml")
    widget.show()

    app.exec()

if __name__ == "__main__":
    normal_execution()
    # test_open()
    # test_save()