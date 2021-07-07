from PyQt5 import QtGui, QtCore, QtWidgets
from FlatlandModel import *
from QtExtensions import *
import numpy as np


class FootprintWidget(QtWidgets.QFrame):
    index = 0

    def __init__(self, polygon: ArenaQGraphicsPolygonItem, scene: QtWidgets.QGraphicsScene, view: QtWidgets.QGraphicsView, **kwargs):
        super().__init__(**kwargs)
        self.index = FootprintWidget.index
        FootprintWidget.index += 1
        self.spin_boxes = []  # 2D array holding QDoubleSpinBox objects, self.spin_boxes[i][1] returns the y value spin box for the i-th point in the polygon, self.spin_boxes[i][0] returns the x value
        self.polygon_item = polygon
        self.polygon_item.footprint_widget = self
        self.gscene = scene
        self.gview = view
        self.dragging_polygon = False
        self.setup_ui()

    def setup_ui(self):
        self.setLayout(QtWidgets.QVBoxLayout())
        self.setFrameStyle(QtWidgets.QFrame.Shape.Box | QtWidgets.QFrame.Shadow.Raised)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Maximum, QtWidgets.QSizePolicy.Policy.Maximum)

        # label and delete button widget
        widget = QtWidgets.QWidget()
        widget.setLayout(QtWidgets.QHBoxLayout())
        widget.setSizePolicy(QtWidgets.QSizePolicy.Policy.Maximum, QtWidgets.QSizePolicy.Policy.Maximum)
        self.layout().addWidget(widget)
        ## label
        self.name_label = QtWidgets.QLabel("Polygon " + str(self.index))
        self.name_label.setFixedHeight(20)
        self.name_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        widget.layout().addWidget(self.name_label)
        ## delete button
        button_delete = QtWidgets.QPushButton("delete")
        button_delete.clicked.connect(self.on_delete_clicked)
        widget.layout().addWidget(button_delete)

        # label and layers line edit widget
        widget = QtWidgets.QWidget()
        widget.setLayout(QtWidgets.QHBoxLayout())
        widget.setSizePolicy(QtWidgets.QSizePolicy.Policy.Maximum, QtWidgets.QSizePolicy.Policy.Maximum)
        self.layout().addWidget(widget)
        ## label
        layers_label = QtWidgets.QLabel("layers:")
        widget.layout().addWidget(layers_label)
        ## line edit
        self.layers_line_edit = QtWidgets.QLineEdit("static")
        widget.layout().addWidget(self.layers_line_edit)

        # frame containing polygon points
        self.points_frame = QtWidgets.QFrame()
        self.points_frame.setLayout(QtWidgets.QVBoxLayout())
        self.layout().addWidget(self.points_frame)

        # buttons for adjusting number of points in polygon
        button_widget = QtWidgets.QWidget()
        button_widget.setLayout(QtWidgets.QHBoxLayout())
        ## +
        button_plus = QtWidgets.QPushButton("+")
        button_plus.clicked.connect(lambda x: self.add_point())
        button_widget.layout().addWidget(button_plus)
        ## -
        button_minus = QtWidgets.QPushButton("-")
        button_minus.clicked.connect(self.remove_point)
        button_widget.layout().addWidget(button_minus)
        self.layout().addWidget(button_widget)

        for point in self.polygon_item.polygon():
            self.add_point(point)

    def add_point(self, point: QtCore.QPointF = None):
        if len(self.polygon_item.polygon()) > 7:
            QtWidgets.QMessageBox.information(self, "Information", "Can't add more edges.\nFlatland only allows up to 8 edges for a single polygon.")
            return

        new_spin_boxes = []

        # widget
        widget = QtWidgets.QWidget()
        widget.setLayout(QtWidgets.QHBoxLayout())
        widget.setSizePolicy(QtWidgets.QSizePolicy.Policy.Maximum, QtWidgets.QSizePolicy.Policy.Maximum)
        self.points_frame.layout().addWidget(widget)

        # x value
        ## label
        x_label = QtWidgets.QLabel("x")
        widget.layout().addWidget(x_label)
        ## spinbox
        x_spinbox = ArenaQDoubleSpinBox()
        x_spinbox.setMinimum(-100.0)
        x_spinbox.setSingleStep(0.1)
        if point != None:
            x_spinbox.setValue(point.x())
        else:
            x_spinbox.setValue(0.0)
        # x_spinbox.valueChanged.connect(lambda value: self.update_point(value, id, 0))
        x_spinbox.valueChanged.connect(self.update_polygon)
        new_spin_boxes.append(x_spinbox)
        widget.layout().addWidget(x_spinbox)

        # y value
        ## label
        y_label = QtWidgets.QLabel("y")
        widget.layout().addWidget(y_label)
        ## spinbox
        y_spinbox = ArenaQDoubleSpinBox()
        y_spinbox.setMinimum(-100.0)
        y_spinbox.setSingleStep(0.1)
        if point != None:
            y_spinbox.setValue(point.y())
        else:
            y_spinbox.setValue(0.0)
        y_spinbox.valueChanged.connect(self.update_polygon)
        new_spin_boxes.append(y_spinbox)
        widget.layout().addWidget(y_spinbox)

        self.spin_boxes.append(new_spin_boxes)
        self.update_polygon()

    def remove_point(self):
        '''
        Removes the point that was added last.
        '''
        layout_ = self.points_frame.layout()
        count = layout_.count()
        if count > 3:
            item = layout_.itemAt(count - 1)
            layout_.removeItem(item)
            self.spin_boxes.pop()
            self.update_polygon()

    def update_polygon(self):
        if not self.dragging_polygon:  # prevents recursive loop (spin box <-> moving item)
            new_points = [QtCore.QPointF(point_boxes[0].value(), point_boxes[1].value()) for point_boxes in self.spin_boxes]
            new_polygon = QtGui.QPolygonF(new_points)
            self.polygon_item.setPolygon(self.polygon_item.mapFromScene(new_polygon))

    def update_spin_boxes(self):
        mapped_polygon = self.polygon_item.mapToScene(self.polygon_item.polygon())
        for i, point in enumerate(mapped_polygon):
            self.spin_boxes[i][0].setValue(point.x())
            self.spin_boxes[i][1].setValue(point.y())

    def on_delete_clicked(self):
        self.parent().layout().removeWidget(self)
        self.gscene.removeItem(self.polygon_item)
        self.deleteLater()


class FlatlandBodyEditor(QtWidgets.QWidget):
    def __init__(self, id, flatland_body, flatland_body_widget, **kwargs):
        super().__init__(**kwargs)
        self.id = id
        self.polygons = []
        self.flatland_body_widget = flatland_body_widget
        self.flatland_body = None
        self.setup_ui()
        self.set_flatland_body(flatland_body)  # needs to be called after setup_ui
    
    def setup_ui(self):
        self.setWindowTitle("Flatland Body Editor")
        self.setLayout(QtWidgets.QGridLayout())
        self.setWindowModality(QtCore.Qt.WindowModality.ApplicationModal)
        self.resize(1000, 600)

        # name
        ## label
        name_label = QtWidgets.QLabel("### Name")
        name_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.layout().addWidget(name_label, 0, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## editbox
        self.name_edit = QtWidgets.QLineEdit("new_body")
        self.name_edit.setFixedSize(150, 30)
        self.layout().addWidget(self.name_edit, 0, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # type
        ## label
        type_label = QtWidgets.QLabel("### Type")
        type_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.layout().addWidget(type_label, 1, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## dropdown
        self.type_dropdown = QtWidgets.QComboBox()
        for body_type in B2BodyType:
            self.type_dropdown.insertItem(body_type.value, body_type.name.lower())
        self.type_dropdown.setFixedSize(150, 30)
        self.layout().addWidget(self.type_dropdown, 1, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # color
        ## label
        color_label = QtWidgets.QLabel("### Color")
        color_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.layout().addWidget(color_label, 2, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        # color dialog
        self.color_dialog = QtWidgets.QColorDialog()
        # button
        self.color_button = QtWidgets.QPushButton("")
        self.color_button.setFixedSize(150, 30)
        self.color_button.setStyleSheet(f"background-color: {self.color_dialog.currentColor().name()}")
        self.color_button.clicked.connect(self.on_color_button_clicked)
        self.layout().addWidget(self.color_button, 2, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # linear damping
        ## label
        linear_damping_label = QtWidgets.QLabel("### Linear Damping")
        linear_damping_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.layout().addWidget(linear_damping_label, 3, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## spinbox
        self.linear_damping_spin_box = QtWidgets.QDoubleSpinBox()
        self.linear_damping_spin_box.setValue(0.0)
        self.linear_damping_spin_box.setSingleStep(0.1)
        self.linear_damping_spin_box.setMinimum(0.0)
        self.linear_damping_spin_box.setFixedSize(150, 30)
        self.layout().addWidget(self.linear_damping_spin_box, 3, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # angular damping
        ## label
        angular_damping_label = QtWidgets.QLabel("### Angular Damping")
        angular_damping_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.layout().addWidget(angular_damping_label, 4, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## spinbox
        self.angular_damping_spin_box = QtWidgets.QDoubleSpinBox()
        self.angular_damping_spin_box.setValue(0.0)
        self.angular_damping_spin_box.setSingleStep(0.1)
        self.angular_damping_spin_box.setMinimum(0.0)
        self.angular_damping_spin_box.setFixedSize(150, 30)
        self.layout().addWidget(self.angular_damping_spin_box, 4, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # drawing frame
        ## frame
        drawing_frame = QtWidgets.QFrame()
        drawing_frame.setLayout(QtWidgets.QVBoxLayout())
        drawing_frame.setFrameStyle(QtWidgets.QFrame.Shape.Box | QtWidgets.QFrame.Shadow.Raised)
        self.layout().addWidget(drawing_frame, 0, 2, 8, 1)
        ## graphicsscene
        self.gscene = QtWidgets.QGraphicsScene()
        ## graphicsview
        self.gview = ArenaQGraphicsView(self.gscene)
        drawing_frame.layout().addWidget(self.gview)

        # add polygon button
        self.add_polygon_button = QtWidgets.QPushButton("Add Polygon")
        self.add_polygon_button.clicked.connect(self.on_add_polygon_clicked)
        self.layout().addWidget(self.add_polygon_button, 6, 0, 1, 2)

        # footprints
        ## scrollarea
        self.footprints_scrollarea = QtWidgets.QScrollArea(self)
        self.footprints_scrollarea.setWidgetResizable(True)
        self.layout().addWidget(self.footprints_scrollarea, 7, 0, 1, 2)
        ## frame
        self.footprints_frame = QtWidgets.QFrame()
        self.footprints_frame.setLayout(QtWidgets.QVBoxLayout())
        spacer = QtWidgets.QSpacerItem(1, 1, vPolicy=QtWidgets.QSizePolicy.Policy.Expanding)
        self.footprints_frame.layout().addSpacerItem(spacer)
        self.footprints_scrollarea.setWidget(self.footprints_frame)

        # save button
        self.save_button = QtWidgets.QPushButton("Save and Close")
        self.save_button.clicked.connect(self.on_save_clicked)
        self.layout().addWidget(self.save_button, 8, 0, 1, -1)


    def on_add_polygon_clicked(self):
        # create default polygon
        polygon = QtGui.QPolygonF(
            [
                QtCore.QPointF(0.5, 0.5),
                QtCore.QPointF(0.5, -0.5),
                QtCore.QPointF(-0.5, -0.5),
                QtCore.QPointF(-0.5, 0.5),
            ]
        )
        self.add_polygon_footprint(polygon, self.color_dialog.currentColor())

    def add_polygon_footprint(self, polygon: QtGui.QPolygonF, color: QtGui.QColor):
        # create brush
        brush = QtGui.QBrush(color, QtCore.Qt.BrushStyle.SolidPattern)
        # create pen
        pen = QtGui.QPen()
        pen.setWidthF(0.01)
        pen.setStyle(QtCore.Qt.PenStyle.SolidLine)
        pen.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(QtCore.Qt.PenJoinStyle.RoundJoin)
        # add polygon to scene
        polygon_item = ArenaQGraphicsPolygonItem(polygon)
        polygon_item.setPen(pen)
        polygon_item.setBrush(brush)
        self.gscene.addItem(polygon_item)

        # add FootprintWidget to list
        self.footprints_frame.layout().insertWidget(0, FootprintWidget(polygon_item, self.gscene, self.gview))

    def get_polygon_items(self):
        polygon_items = []
        for i in range(self.footprints_frame.layout().count()):
            w = self.footprints_frame.layout().itemAt(i).widget()
            if w != None:
                polygon_items.append(w.polygon_item)
        return polygon_items

    def get_footprint_widgets(self):
        widgets = []
        for i in range(self.footprints_frame.layout().count()):
            w = self.footprints_frame.layout().itemAt(i).widget()
            if w != None:
                widgets.append(w)
        return widgets

    def on_color_button_clicked(self):
        # get color
        color_selected = self.color_dialog.getColor()
        if QtGui.QColor.isValid(color_selected):
            self.set_color(color_selected)

    def on_save_clicked(self):
        # save body
        self.save()
        # update name label in parent
        self.flatland_body_widget.name_label.setText(self.flatland_body.name)
        # hide window
        self.hide()

    def save(self):
        self.update_body_from_widgets(self.flatland_body)

    def get_body_from_widgets(self):
        body = FlatlandBody()
        self.update_body_from_widgets(body)
        return body

    def update_body_from_widgets(self, body: FlatlandBody):
        # name
        body.name = self.name_edit.text()
        # type
        body.type = B2BodyType(self.type_dropdown.currentIndex())
        # color
        body.color = self.color_dialog.currentColor()
        # linear damping
        body.linear_damping = self.linear_damping_spin_box.value()
        # angular damping
        body.angular_damping = self.angular_damping_spin_box.value()
        # footprints
        body.footprints = []
        for w in self.get_footprint_widgets():
            polygon_item = w.polygon_item
            footprint = PolygonFlatlandFootprint()
            # remove all whitespace and split on ","
            footprint.layers = "".join(w.layers_line_edit.text().split()).split(",")
            footprint.points = [[point.x(), point.y()] for point in polygon_item.mapToScene(polygon_item.polygon())]
            body.footprints.append(footprint)

    def closeEvent(self, event):
        # check if any changes have been made
        current_body = self.get_body_from_widgets()
        if self.flatland_body != current_body:
            # ask user if she wants to save changes
            msg_box = QtWidgets.QMessageBox()
            msg_box.setText("Do you want to save changes to this body?")
            msg_box.setStandardButtons(QtWidgets.QMessageBox.Save | QtWidgets.QMessageBox.Discard | QtWidgets.QMessageBox.Cancel)
            msg_box.setDefaultButton(QtWidgets.QMessageBox.Save)
            ret = msg_box.exec()
            if ret == QtWidgets.QMessageBox.Save:
                self.on_save_clicked()
            elif ret == QtWidgets.QMessageBox.Discard:
                # reset to values already saved
                self.set_flatland_body(self.flatland_body)
            elif ret == QtWidgets.QMessageBox.Cancel:
                event.ignore()

    def set_color(self, color: QtGui.QColor):
        # change button color
        self.color_button.setStyleSheet(f"background-color: {color.name()}")
        self.color_dialog.setCurrentColor(color)
        # change polygon color
        for polygon_item in self.get_polygon_items():
            brush = QtGui.QBrush(color, QtCore.Qt.BrushStyle.SolidPattern)
            polygon_item.setBrush(brush)

    def set_flatland_body(self, body: FlatlandBody):
        self.flatland_body = body

        # set name
        self.name_edit.setText(body.name)

        # set type
        self.type_dropdown.setCurrentIndex(body.type.value)

        # set color
        self.set_color(body.color)

        # set damping
        self.linear_damping_spin_box.setValue(body.linear_damping)
        self.angular_damping_spin_box.setValue(body.angular_damping)

        # set footprints
        # remove all widgets
        for w in self.get_footprint_widgets():
            w.on_delete_clicked()
        # add new polygons and widgets
        for fp in self.flatland_body.footprints:
            if isinstance(fp, PolygonFlatlandFootprint):
                polygon = QtGui.QPolygonF([QtCore.QPointF(point[0], point[1]) for point in fp.points])
                self.add_polygon_footprint(polygon, body.color)
            elif isinstance(fp, CircleFlatlandFootprint):
                # TODO
                pass
        if self.flatland_body_widget != None:
            self.flatland_body_widget.name_label.setText(self.flatland_body.name)

if __name__ == "__main__":
    app = QtWidgets.QApplication([])

    model = FlatlandModel()
    body = FlatlandBody()
    model.bodies[0] = body
    widget = FlatlandBodyEditor(0, model.bodies[0], None)
    widget.on_add_polygon_clicked()
    # widget.on_add_polygon_clicked()
    widget.show()

    app.exec()
