from PyQt5 import QtGui, QtCore, QtWidgets
from FlatlandModel import FlatlandBody, B2BodyType, FlatlandModel

class FlatlandBodyEditor(QtWidgets.QWidget):
    def __init__(self, id, model, **kwargs):
        super().__init__(**kwargs)
        self.id = id
        self.flatland_body = FlatlandBody()
        model.bodies[id] = self.flatland_body
        self.setup_ui()
    
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
        self.name_edit = QtWidgets.QLineEdit("new_body")
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

        # color
        ## label
        color_label = QtWidgets.QLabel("### Color")
        color_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.layout.addWidget(color_label, 2, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        # color dialog
        self.color_dialog = QtWidgets.QColorDialog()
        self.color = self.flatland_body.color
        # button
        self.color_button = QtWidgets.QPushButton("")
        self.color_button.setFixedSize(150, 30)
        self.color_button.setStyleSheet(f"background-color: {self.flatland_body.color.name()}")
        self.color_button.clicked.connect(self.on_color_button_clicked)
        self.layout.addWidget(self.color_button, 2, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # linear damping
        ## label
        linear_damping_label = QtWidgets.QLabel("### Linear Damping")
        linear_damping_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.layout.addWidget(linear_damping_label, 3, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## spinbox
        self.linear_damping_spin_box = QtWidgets.QDoubleSpinBox()
        self.linear_damping_spin_box.setValue(0.0)
        self.linear_damping_spin_box.setSingleStep(0.1)
        self.linear_damping_spin_box.setMinimum(0.0)
        self.linear_damping_spin_box.setFixedSize(150, 30)
        self.layout.addWidget(self.linear_damping_spin_box, 3, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # angular damping
        ## label
        angular_damping_label = QtWidgets.QLabel("### Angular Damping")
        angular_damping_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.layout.addWidget(angular_damping_label, 4, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## spinbox
        self.angular_damping_spin_box = QtWidgets.QDoubleSpinBox()
        self.angular_damping_spin_box.setValue(0.0)
        self.angular_damping_spin_box.setSingleStep(0.1)
        self.angular_damping_spin_box.setMinimum(0.0)
        self.angular_damping_spin_box.setFixedSize(150, 30)
        self.layout.addWidget(self.angular_damping_spin_box, 4, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # drawing frame
        # self.drawing_frame = QtWidgets.QFrame()
        self.scene = QtWidgets.QGraphicsScene()
        self.scene.addText("hello\n\nworld")

        self.myview = QtWidgets.QGraphicsView(self.scene)
        self.layout.addWidget(self.myview, 5, 0, 1, -1)
        # self.myview.show()

        # self.drawing_frame = QtWidgets.QGraphicsView()
        # self.drawing_frame.setMinimumHeight(100)
        # self.layout.addWidget(self.drawing_frame)

        points = QtGui.QPolygonF(
            [
                QtCore.QPointF(10.0, 80.0),
                QtCore.QPointF(20.0, 10.0),
                QtCore.QPointF(80.0, 30.0),
                QtCore.QPointF(90.0, 70.0)
            ]
        )

        self.scene.addPolygon(points)

        # painter = QtGui.QPainter(self.drawing_frame)
        # painter.setRenderHint(QtGui.QPainter.Antialiasing, False)
        # painter.setPen(self.palette().dark().color())
        # painter.setBrush(QtCore.Qt.NoBrush)
        # painter.drawRect(QtCore.QRect(0, 0, self.width() - 1, self.height() - 1))

        # painter.drawPolygon(points, 4)


        # save button
        self.save_button = QtWidgets.QPushButton("Save and Close")
        self.save_button.clicked.connect(self.on_save_clicked)
        self.layout.addWidget(self.save_button, 6, 0, 1, -1)

        self.resize(500, 200)

    def on_color_button_clicked(self):
        # get color
        color_selected = self.color_dialog.getColor()
        if QtGui.QColor.isValid(color_selected):
            self.color = color_selected
            # change button color
            self.color_button.setStyleSheet(f"background-color: {self.color.name()}")

    def on_save_clicked(self):
        # name
        self.flatland_body.name = self.name_edit.text()
        # type
        self.flatland_body.type = B2BodyType(self.type_dropdown.currentIndex())
        # color
        self.flatland_body.color = self.color
        # linear damping
        self.flatland_body.linear_damping = self.linear_damping_spin_box.value()
        # angular damping
        self.flatland_body.angular_damping = self.angular_damping_spin_box.value()
        # update name label in parent
        self.parent().name_label.setText(self.flatland_body.name)
        # hide window
        self.hide()

    # def paintEvent(self, a0: QtGui.QPaintEvent) -> None:
    #     painter = QtGui.QPainter(self.drawing_frame)
    #     # painter = QtGui.QPainter(self)
    #     painter.setRenderHint(QtGui.QPainter.Antialiasing, False)
    #     painter.setPen(self.palette().dark().color())
    #     painter.setBrush(QtCore.Qt.NoBrush)
    #     painter.drawRect(QtCore.QRect(0, 0, self.width() - 10, self.height() - 10))

    #     # return super().paintEvent(a0)
    

    def closeEvent(self, event):
        # reset to values already saved
        self.name_edit.setText(self.flatland_body.name)
        self.type_dropdown.setCurrentIndex(self.flatland_body.type.value)
        self.color_button.setStyleSheet(f"background-color: {self.flatland_body.color.name()}")
        self.linear_damping_spin_box.setValue(self.flatland_body.linear_damping)
        self.angular_damping_spin_box.setValue(self.flatland_body.angular_damping)

if __name__ == "__main__":
    app = QtWidgets.QApplication([])

    model = FlatlandModel()
    widget = FlatlandBodyEditor(0, model)
    widget.show()

    app.exec()
