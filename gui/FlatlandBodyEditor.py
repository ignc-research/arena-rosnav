from PyQt5 import QtGui, QtCore, QtWidgets
from enum import Enum

class B2BodyType(Enum):
    DYNAMIC = 0
    STATIC = 1
    KINEMATIC = 2


class FlatlandBody():
    def __init__(self):
        self.name = "new_body"
        self.type = B2BodyType.DYNAMIC
        self.color = None
        self.linear_damping = 0.0
        self.angular_damping = 0.0


class FlatlandBodyEditor(QtWidgets.QWidget):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.setup_ui()
        self.flatland_body = FlatlandBody()
    
    def setup_ui(self):
        self.setWindowTitle("Flatland Body Editor")

        self.layout = QtWidgets.QGridLayout()
        self.setLayout(self.layout)

        # name
        ## label
        name_label = QtWidgets.QLabel("### Name")
        name_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.layout.addWidget(name_label, 0, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## editbox
        self.name_edit = QtWidgets.QLineEdit()
        self.name_edit.setFixedSize(150, 30)
        self.layout.addWidget(self.name_edit, 0, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # type
        ## label
        type_label = QtWidgets.QLabel("### Type")
        type_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.layout.addWidget(type_label, 1, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## dropdown
        self.type_dropdown = QtWidgets.QComboBox()
        for body_type in B2BodyType:
            self.type_dropdown.insertItem(body_type.value, body_type.name.lower())
        self.type_dropdown.setFixedSize(150, 30)
        self.layout.addWidget(self.type_dropdown, 1, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # TODO color

        # linear damping
        ## label
        linear_damping_label = QtWidgets.QLabel("### Linear Damping")
        linear_damping_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.layout.addWidget(linear_damping_label, 2, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## spinbox
        self.linear_damping_spin_box = QtWidgets.QDoubleSpinBox()
        self.linear_damping_spin_box.setValue(0.0)
        self.linear_damping_spin_box.setSingleStep(0.1)
        self.linear_damping_spin_box.setMinimum(0.0)
        self.linear_damping_spin_box.setFixedSize(150, 30)
        self.layout.addWidget(self.linear_damping_spin_box, 2, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # angular damping
        ## label
        angular_damping_label = QtWidgets.QLabel("### Angular Damping")
        angular_damping_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.layout.addWidget(angular_damping_label, 3, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## spinbox
        self.angular_damping_spin_box = QtWidgets.QDoubleSpinBox()
        self.angular_damping_spin_box.setValue(0.0)
        self.angular_damping_spin_box.setSingleStep(0.1)
        self.angular_damping_spin_box.setMinimum(0.0)
        self.angular_damping_spin_box.setFixedSize(150, 30)
        self.layout.addWidget(self.angular_damping_spin_box, 3, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # save button
        self.save_button = QtWidgets.QPushButton("Save and Close")
        self.save_button.clicked.connect(self.on_save_clicked)
        self.layout.addWidget(self.save_button, 4, 0, 1, -1)

        self.resize(500, 200)

    def on_save_clicked(self):
        # name
        self.flatland_body.name = self.name_edit.text()
        # type
        self.flatland_body.type = B2BodyType(self.type_dropdown.currentIndex())
        # color
        # TODO
        # linear damping
        self.flatland_body.linear_damping = self.linear_damping_spin_box.value()
        # angular damping
        self.flatland_body.angular_damping = self.angular_damping_spin_box.value()
        # update name label in parent
        self.parent().name_label.setText(self.flatland_body.name)
        # hide window
        self.hide()

    def closeEvent(self, event):
        # reset to values already saved
        self.name_edit.setText(self.flatland_body.name)
        self.type_dropdown.setCurrentIndex(self.flatland_body.type.value)
        self.linear_damping_spin_box.setValue(self.flatland_body.linear_damping)
        self.angular_damping_spin_box.setValue(self.flatland_body.angular_damping)

if __name__ == "__main__":
    app = QtWidgets.QApplication([])

    widget = FlatlandBodyEditor()
    widget.show()

    app.exec()
