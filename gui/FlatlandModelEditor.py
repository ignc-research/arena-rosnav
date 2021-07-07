from PyQt5 import QtGui, QtCore, QtWidgets
import rospkg
import os
import copy
from FlatlandModel import *
from FlatlandBodyEditor import FlatlandBodyEditor

class FlatlandBodyWidget(QtWidgets.QWidget):
    '''
    This is a row in the bodies frame. It has the following elements:
        - body name label
        - edit button
        - delete button
    '''
    def __init__(self, id: int, flatland_model: FlatlandModel, create_new_flatland_body = True, **kwargs):
        super().__init__(**kwargs)
        self.setup_ui()
        self.id = id
        self.model = flatland_model
        if create_new_flatland_body:
            flatland_body = FlatlandBody()
            self.model.bodies[id] = flatland_body

        self.flatland_body_editor = FlatlandBodyEditor(id, self.model.bodies[id], self, parent=self.parent(), flags=QtCore.Qt.WindowType.Window)

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
        self.model = None
        self.last_saved_model = None
        self.body_id = 0
        self.create_new_model()

    def setup_ui(self):
        self.setWindowTitle("Flatland Model Editor")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap('icon.png'), QtGui.QIcon.Selected, QtGui.QIcon.On)
        self.setWindowIcon(icon)

        central_widget = QtWidgets.QWidget()
        central_widget.setLayout(QtWidgets.QVBoxLayout())
        self.setCentralWidget(central_widget)
        menubar = self.menuBar()
        file_menu = menubar.addMenu("File")
        file_menu.addAction("New Model", self.on_new_model_clicked, "Ctrl+N")
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
        self.body_id = 0

    def on_new_model_clicked(self):
        if self.last_saved_model != self.model and len(self.model.bodies) > 0:
            # ask user if she wants to save changes
            msg_box = QtWidgets.QMessageBox()
            msg_box.setText("Do you want to save changes?")
            msg_box.setStandardButtons(QtWidgets.QMessageBox.Save | QtWidgets.QMessageBox.Discard | QtWidgets.QMessageBox.Cancel)
            msg_box.setDefaultButton(QtWidgets.QMessageBox.Save)
            ret = msg_box.exec()
            if ret == QtWidgets.QMessageBox.Save:
                if not self.on_save_as_clicked():
                    # save as dialog was cancelled
                    return
            elif ret == QtWidgets.QMessageBox.Discard:
                pass
            elif ret == QtWidgets.QMessageBox.Cancel:
                return
        self.create_new_model()

    def create_new_model(self):
        self.remove_all_bodies()
        self.model = FlatlandModel()

    def on_add_body_button_clicked(self):
        w = FlatlandBodyWidget(self.body_id, self.model, parent=self)
        return self.add_flatland_body_widget(w)

    def add_flatland_body_widget(self, w: FlatlandBodyWidget):
        self.bodies_list_frame.layout().addWidget(w)
        self.body_id += 1
        return w

    def on_open_clicked(self):
        rospack = rospkg.RosPack()
        initial_folder = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles")
        res = QtWidgets.QFileDialog.getOpenFileName(parent=self, directory=initial_folder)
        path = res[0]
        if path != "":
            self.load_model(path)

    def load_model(self, path):
        self.setWindowTitle(path.split("/")[-1])
        self.remove_all_bodies()
        self.model.load(path)
        for _ in self.model.bodies.values():
            w = FlatlandBodyWidget(self.body_id, self.model, create_new_flatland_body=False, parent=self)
            self.add_flatland_body_widget(w)

        self.last_saved_model = copy.deepcopy(self.model)

    def on_save_clicked(self):
        if self.model.save():
           self.last_saved_model = copy.deepcopy(self.model)
        else:
            # no path has been set yet. fall back to "save as"
            if self.on_save_as_clicked():
                self.last_saved_model = copy.deepcopy(self.model)

    def on_save_as_clicked(self):
        rospack = rospkg.RosPack()
        initial_folder = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles")

        res = QtWidgets.QFileDialog.getSaveFileName(parent=self, directory=initial_folder)
        path = res[0]
        if path != "":
            self.model.save(path)
            self.last_saved_model = copy.deepcopy(self.model)
            return True
        else:
            return False

    def closeEvent(self, event: QtGui.QCloseEvent):
        if self.last_saved_model != self.model and len(self.model.bodies) > 0:
            # ask user if she wants to save changes
            msg_box = QtWidgets.QMessageBox()
            msg_box.setText("Do you want to save changes?")
            msg_box.setStandardButtons(QtWidgets.QMessageBox.Save | QtWidgets.QMessageBox.Discard | QtWidgets.QMessageBox.Cancel)
            msg_box.setDefaultButton(QtWidgets.QMessageBox.Save)
            ret = msg_box.exec()
            if ret == QtWidgets.QMessageBox.Save:
                self.on_save_as_clicked()
            elif ret == QtWidgets.QMessageBox.Discard:
                pass
            elif ret == QtWidgets.QMessageBox.Cancel:
                event.ignore()

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
    widget.load_model("/home/daniel/catkin_ws/src/arena-rosnav/simulator_setup/obstacles/cleaner.model.yaml")
    widget.show()

    app.exec()

if __name__ == "__main__":
    normal_execution()
    # test_open()
    # test_save()