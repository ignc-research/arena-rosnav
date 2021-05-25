from PyQt5 import QtGui, QtCore, QtWidgets
from FlatlandBodyEditor import FlatlandBodyEditor

class FlatlandBodyWidget(QtWidgets.QWidget):
    '''
    This is a row in the bodies frame. It has the following elements:
        - body name label
        - edit button
        - delete button
    '''
    def __init__(self):
        super().__init__()
        self.setup_ui()
        self.flatland_body_editor = FlatlandBodyEditor(parent=self, flags=QtCore.Qt.WindowType.Window)

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
        self.parent().layout().removeWidget(self)
        self.deleteLater()


class FlatlandModelEditor(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setup_ui()

    def setup_ui(self):
        self.setWindowTitle("Flatland Model Editor")
        self.layout = QtWidgets.QVBoxLayout()
        self.setLayout(self.layout)

        # test
        self.test_button = QtWidgets.QPushButton("save")
        self.test_button.clicked.connect(self.on_test_clicked)
        self.layout.addWidget(self.test_button)

        # add frame to add and edit bodies
        self.setup_bodies_frame()

        # add expanding spacer item
        spacer = QtWidgets.QSpacerItem(1, 1, vPolicy=QtWidgets.QSizePolicy.Policy.Expanding)
        self.layout.addSpacerItem(spacer)

        self.resize(500, 200)

    def setup_bodies_frame(self):
        frame = QtWidgets.QFrame()
        layout = QtWidgets.QGridLayout()
        frame.setLayout(layout)

        # add title
        bodies_label = QtWidgets.QLabel("## Bodies")
        bodies_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        layout.addWidget(bodies_label, 0, 0, QtCore.Qt.AlignmentFlag.AlignLeft)

        # add "add body" button
        self.add_body_button = QtWidgets.QPushButton("Add Body")
        self.add_body_button.setFixedSize(200, 30)
        self.add_body_button.clicked.connect(self.on_add_body_button_clicked)
        layout.addWidget(self.add_body_button, 0, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # add body list
        self.bodies_list_frame = QtWidgets.QFrame()
        self.bodies_list_frame.setLayout(QtWidgets.QVBoxLayout())
        layout.addWidget(self.bodies_list_frame, 1, 0, 1, -1)

        # add this frame to layout
        self.layout.addWidget(frame)

    def on_add_body_button_clicked(self):
        self.bodies_list_frame.layout().addWidget(FlatlandBodyWidget())

    def on_test_clicked(self):
        for i in range(self.bodies_list_frame.layout().count()):
            widget = self.bodies_list_frame.layout().itemAt(i)
            print(widget.widget().flatland_body_editor.flatland_body.name, i)


if __name__ == "__main__":
    app = QtWidgets.QApplication([])

    widget = FlatlandModelEditor()
    widget.show()

    app.exec()