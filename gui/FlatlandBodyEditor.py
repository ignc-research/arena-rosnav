from PyQt5 import QtGui, QtCore, QtWidgets
from FlatlandModel import FlatlandBody, B2BodyType, FlatlandModel
import random

class ArenaQGraphicsView(QtWidgets.QGraphicsView):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setDragMode(QtWidgets.QGraphicsView.DragMode.ScrollHandDrag)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        self.setSceneRect(-100, -100, 200, 200)

        # add coordinate system lines
        pen = QtGui.QPen()
        pen.setWidthF(0.1)
        self.scene().addLine(0, -100, 0, 100, pen)
        self.scene().addLine(-100, 0, 100, 0, pen)

        # set initial view
        rect = QtCore.QRectF(-20, -20, 40, 40)
        self.fitInView(rect, mode=QtCore.Qt.AspectRatioMode.KeepAspectRatio)

    def wheelEvent(self, event):
        """
        Zoom in or out of the view.
        """
        zoomInFactor = 1.25
        zoomOutFactor = 1 / zoomInFactor

        # Save the scene pos
        oldPos = self.mapToScene(event.pos())

        # Zoom
        if event.angleDelta().y() > 0:
            zoomFactor = zoomInFactor
        else:
            zoomFactor = zoomOutFactor
        self.scale(zoomFactor, zoomFactor)

        # Get the new position
        newPos = self.mapToScene(event.pos())

        # Move scene to old position
        delta = newPos - oldPos
        self.translate(delta.x(), delta.y())


class FootprintWidget(QtWidgets.QFrame):
    index = 0

    def __init__(self, polygon: QtWidgets.QGraphicsPolygonItem, scene: QtWidgets.QGraphicsScene, view: QtWidgets.QGraphicsView, **kwargs):
        super().__init__(**kwargs)
        self.index = FootprintWidget.index
        FootprintWidget.index += 1
        self.points = [[point.x(), point.y()] for point in polygon.polygon()]  # 2D array
        self.polygon = polygon
        self.gscene = scene
        self.gview = view
        self.setup_ui()

    def setup_ui(self):
        self.setLayout(QtWidgets.QVBoxLayout())
        self.setFrameStyle(QtWidgets.QFrame.Shape.Box | QtWidgets.QFrame.Shadow.Raised)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Maximum, QtWidgets.QSizePolicy.Policy.Maximum)

        # label
        self.name_label = QtWidgets.QLabel("Polygon " + str(self.index))
        self.name_label.setFixedHeight(20)
        self.name_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.layout().addWidget(self.name_label)

        self.points_frame = QtWidgets.QFrame()
        self.points_frame.setLayout(QtWidgets.QVBoxLayout())
        self.layout().addWidget(self.points_frame)

        button = QtWidgets.QPushButton("print points")
        button.clicked.connect(lambda x: print(self.points))
        self.layout().addWidget(button)

        for i, point in enumerate(self.points):
            self.add_point(i, point)

    def add_point(self, id, point=None):
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
        x_spinbox = QtWidgets.QDoubleSpinBox()
        if point != None:
            x_spinbox.setValue(point[0])
        else:
            x_spinbox.setValue(0.0)
        x_spinbox.setMinimum(-100.0)
        x_spinbox.setSingleStep(0.1)
        x_spinbox.valueChanged.connect(lambda value: self.update_point(value, id, 0))
        widget.layout().addWidget(x_spinbox)

        # y value
        ## label
        y_label = QtWidgets.QLabel("y")
        widget.layout().addWidget(y_label)
        ## spinbox
        y_spinbox = QtWidgets.QDoubleSpinBox()
        if point != None:
            y_spinbox.setValue(point[1])
        else:
            y_spinbox.setValue(0.0)
        y_spinbox.setMinimum(-100.0)
        y_spinbox.setSingleStep(0.1)
        y_spinbox.valueChanged.connect(lambda value: self.update_point(value, id, 1))
        widget.layout().addWidget(y_spinbox)

        if point == None:
            point = [0.0, 0.0]
            self.points.append(point)

    def update_point(self, value, point_id, point_index):
        self.points[point_id][point_index] = value
        self.update_polygon()
        

    def update_polygon(self):
        new_points = [QtCore.QPointF(element[0], element[1]) for element in self.points]
        new_polygon = QtGui.QPolygonF(new_points)
        self.polygon.setPolygon(new_polygon)


class FlatlandBodyEditor(QtWidgets.QWidget):
    def __init__(self, id, model, **kwargs):
        super().__init__(**kwargs)
        self.id = id
        self.flatland_body = FlatlandBody()
        model.bodies[id] = self.flatland_body
        self.polygons = []
        self.setup_ui()
    
    def setup_ui(self):
        self.setWindowTitle("Flatland Body Editor")
        self.setLayout(QtWidgets.QGridLayout())
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
        self.color = self.flatland_body.color
        # button
        self.color_button = QtWidgets.QPushButton("")
        self.color_button.setFixedSize(150, 30)
        self.color_button.setStyleSheet(f"background-color: {self.flatland_body.color.name()}")
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
                QtCore.QPointF(0.0, 0.0),
                QtCore.QPointF(10.0, 0.0),
                QtCore.QPointF(10.0, 10.0),
                QtCore.QPointF(0.0, 10.0),
            ]
        )
        # create brush
        brush = QtGui.QBrush(QtGui.QColor("black"), QtCore.Qt.BrushStyle.SolidPattern)
        # create pen
        pen = QtGui.QPen()
        pen.setWidthF(0.5)
        pen.setStyle(QtCore.Qt.PenStyle.SolidLine)
        pen.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(QtCore.Qt.PenJoinStyle.RoundJoin)
        # add polygon to scene
        polygon_item = self.gscene.addPolygon(polygon, pen, brush)
        polygon_item.setFlags(QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsMovable | QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)
        # fit to view
        # rect = self.gscene.itemsBoundingRect() + QtCore.QMarginsF(-20, -20, 40, 40)
        # self.gview.fitInView(rect, mode=QtCore.Qt.AspectRatioMode.KeepAspectRatio)

        # add FootprintWidget to list
        self.footprints_frame.layout().insertWidget(0, FootprintWidget(polygon_item, self.gscene, self.gview))

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
    widget.on_add_polygon_clicked()
    # widget.on_add_polygon_clicked()
    widget.show()

    app.exec()
