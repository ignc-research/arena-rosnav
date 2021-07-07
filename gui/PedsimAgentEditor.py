from typing import Any
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtCore import Qt, QRect
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QGridLayout, QVBoxLayout, QFrame, QGraphicsScene, QGraphicsView, QScrollArea, QHBoxLayout, QPushButton, QSpacerItem, QSizePolicy, QLabel, QLineEdit, QComboBox, QSpinBox, QSlider
import rospkg
import os
from FlatlandBodyEditor import *
from ArenaScenario import *
from QtExtensions import *

class PedsimAgentEditor(QWidget):
    def __init__(self, pedsimAgentWidget, **kwargs):
        super().__init__(**kwargs)
        self.pedsimAgent = pedsimAgentWidget.pedsimAgent
        self.pedsimAgentWidget = pedsimAgentWidget
        self.tempFlatlandModel = FlatlandModel()
        self.setup_ui()
        self.updateValuesFromPedsimAgent()
        self.updateWidgetsFromSelectedType()

    def setup_ui(self):
        self.setWindowTitle("Pedsim Agent Editor")
        self.setLayout(QtWidgets.QGridLayout())
        self.setWindowModality(QtCore.Qt.WindowModality.ApplicationModal)
        self.resize(600, 600)

        # scrollarea
        self.scrollArea = QScrollArea(self)
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setMinimumWidth(400)
        self.layout().addWidget(self.scrollArea, 0, 0, 1, -1)
        # frame
        self.scrollAreaFrame = QFrame()
        self.scrollAreaFrame.setLayout(QGridLayout())
        self.scrollAreaFrame.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Maximum)
        self.scrollAreaFrame.setMinimumWidth(400)
        self.scrollArea.setWidget(self.scrollAreaFrame)

        # heading "general"
        general_label = QLabel("#### General")
        general_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(general_label, 0, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        line = Line()
        self.scrollAreaFrame.layout().addWidget(line, 0, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # name
        ## label
        name_label = QLabel("Name")
        name_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(name_label, 1, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## editbox
        self.name_edit = QLineEdit(self.pedsimAgentWidget.name_label.text())
        self.name_edit.setFixedSize(200, 30)
        self.scrollAreaFrame.layout().addWidget(self.name_edit, 1, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # model file
        ## label
        name_label = QLabel("Flatland Model")
        name_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(name_label, 2, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## choose button
        self.modelButton = QPushButton("Choose...")
        self.modelButton.setFixedSize(200, 30)
        self.modelButton.clicked.connect(self.onModelButtonClicked)
        self.scrollAreaFrame.layout().addWidget(self.modelButton, 2, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # type
        ## label
        type_label = QLabel("Type")
        type_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(type_label, 3, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## dropdown
        self.type_dropdown = QComboBox()
        for agent_type in PedsimAgentType:
            self.type_dropdown.insertItem(agent_type.value, agent_type.name.lower())
        self.type_dropdown.setFixedSize(200, 30)
        self.type_dropdown.currentIndexChanged.connect(self.updateWidgetsFromSelectedType)
        self.scrollAreaFrame.layout().addWidget(self.type_dropdown, 3, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # amount
        ## label
        amount_label = QLabel("Amount")
        amount_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(amount_label, 4, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## spin box
        amount_spin_box = QSpinBox()
        amount_spin_box.setValue(1)
        amount_spin_box.setMinimum(1)
        amount_spin_box.setFixedSize(200, 30)
        self.scrollAreaFrame.layout().addWidget(amount_spin_box, 4, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # waypoint mode
        ## label
        label = QLabel("Waypoint Mode")
        label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(label, 5, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## dropdown
        self.waypoint_mode_dropdown = QComboBox()
        for mode in PedsimWaypointMode:
            self.waypoint_mode_dropdown.insertItem(mode.value, mode.name.lower())
        self.waypoint_mode_dropdown.setFixedSize(200, 30)
        self.scrollAreaFrame.layout().addWidget(self.waypoint_mode_dropdown, 5, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # vmax
        ## label
        self.vmax_label = QLabel("Velocity<sub>max</sub>")
        self.vmax_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.vmax_label, 6, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## slider
        self.vmax_slider = ArenaSliderWidget(0, 20, 0.1, "m/s")
        self.scrollAreaFrame.layout().addWidget(self.vmax_slider, 6, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # max talking distance
        ### label
        self.maxTalkingDistanceLabel = QLabel("Max Talking Distance")
        self.maxTalkingDistanceLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.maxTalkingDistanceLabel, 7, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.maxTalkingDistanceSlider = ArenaSliderWidget(0, 10, 1, "m")
        self.scrollAreaFrame.layout().addWidget(self.maxTalkingDistanceSlider, 7, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # forces
        ## heading
        label = QLabel("#### Force Factors")
        label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(label, 8, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        line = Line()
        self.scrollAreaFrame.layout().addWidget(line, 8, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        ## Desired
        ### label
        label = QLabel("Desired")
        label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(label, 9, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        slider = ArenaSliderWidget(0, 20, 1, "")
        slider.slider.setValue(1)
        self.scrollAreaFrame.layout().addWidget(slider, 9, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        ## Obstacle
        ### label
        label = QLabel("Obstacle")
        label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(label, 10, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        slider = ArenaSliderWidget(0, 20, 1, "")
        slider.slider.setValue(1)
        self.scrollAreaFrame.layout().addWidget(slider, 10, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        ## Social
        ### label
        label = QLabel("Social")
        label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(label, 11, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        slider = ArenaSliderWidget(0, 20, 1, "")
        slider.slider.setValue(1)
        self.scrollAreaFrame.layout().addWidget(slider, 11, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        ## Robot
        ### label
        label = QLabel("Robot")
        label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(label, 12, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        slider = ArenaSliderWidget(0, 20, 1, "")
        slider.slider.setValue(1)
        self.scrollAreaFrame.layout().addWidget(slider, 12, 1, QtCore.Qt.AlignmentFlag.AlignRight)


        # individual talking
        ## heading
        self.individualTalkingLabel = QLabel("#### Individual Talking")
        self.individualTalkingLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.individualTalkingLabel, 13, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        self.individualTalkingLine = Line()
        self.scrollAreaFrame.layout().addWidget(self.individualTalkingLine, 13, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        ## probability
        ### label
        self.individualTalkingProbabilityLabel = QLabel("Probability")
        self.individualTalkingProbabilityLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.individualTalkingProbabilityLabel, 14, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.individualTalkingProbabilitySlider = ArenaProbabilitySliderWidget()
        self.scrollAreaFrame.layout().addWidget(self.individualTalkingProbabilitySlider, 14, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        ## base time
        ### label
        self.individualTalkingBaseTimeLabel = QLabel("Base Time")
        self.individualTalkingBaseTimeLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.individualTalkingBaseTimeLabel, 15, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.individualTalkingBaseTimeSlider = ArenaSliderWidget(0, 100, 1, "s")
        self.scrollAreaFrame.layout().addWidget(self.individualTalkingBaseTimeSlider, 15, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # group talk
        ## heading
        self.groupTalkLabel = QLabel("#### Group Talk")
        self.groupTalkLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.groupTalkLabel, 16, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        self.groupTalkLine = Line()
        self.scrollAreaFrame.layout().addWidget(self.groupTalkLine, 16, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        ## probability
        ### label
        self.groupTalkProbabilityLabel = QLabel("Probability")
        self.groupTalkProbabilityLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.groupTalkProbabilityLabel, 17, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.groupTalkProbabilitySlider = ArenaProbabilitySliderWidget()
        self.scrollAreaFrame.layout().addWidget(self.groupTalkProbabilitySlider, 17, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        ## base time
        ### label
        self.groupTalkBaseTimeLabel = QLabel("Base Time")
        self.groupTalkBaseTimeLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.groupTalkBaseTimeLabel, 18, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.groupTalkBaseTimeSlider = ArenaSliderWidget(0, 100, 1, "s")
        self.scrollAreaFrame.layout().addWidget(self.groupTalkBaseTimeSlider, 18, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # talk and walk
        ## heading
        self.talkWalkLabel = QLabel("#### Talk & Walk")
        self.talkWalkLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.talkWalkLabel, 19, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        self.talkWalkLine = Line()
        self.scrollAreaFrame.layout().addWidget(self.talkWalkLine, 19, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        ## probability
        ### label
        self.talkWalkProbabilityLabel = QLabel("Probability")
        self.talkWalkProbabilityLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.talkWalkProbabilityLabel, 20, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.talkWalkProbabilitySlider = ArenaProbabilitySliderWidget()
        self.scrollAreaFrame.layout().addWidget(self.talkWalkProbabilitySlider, 20, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        ## base time
        ### label
        self.talkWalkBaseTimeLabel = QLabel("Base Time")
        self.talkWalkBaseTimeLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.talkWalkBaseTimeLabel, 21, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.talkWalkBaseTimeSlider = ArenaSliderWidget(0, 100, 1, "s")
        self.scrollAreaFrame.layout().addWidget(self.talkWalkBaseTimeSlider, 21, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # requesting service
        ## heading
        self.requestingServiceLabel = QLabel("#### Requesting Service")
        self.requestingServiceLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.requestingServiceLabel, 22, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        self.requestingServiceLine = Line()
        self.scrollAreaFrame.layout().addWidget(self.requestingServiceLine, 22, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        ## probability
        ### label
        self.requestingServiceProbabilityLabel = QLabel("Probability")
        self.requestingServiceProbabilityLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.requestingServiceProbabilityLabel, 23, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.requestingServiceProbabilitySlider = ArenaProbabilitySliderWidget()
        self.scrollAreaFrame.layout().addWidget(self.requestingServiceProbabilitySlider, 23, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        ## max service distance
        ### label
        self.maxServiceDistanceLabel = QLabel("Max Service Distance")
        self.maxServiceDistanceLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.maxServiceDistanceLabel, 24, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.maxServiceDistanceSlider = ArenaSliderWidget(0, 10, 1, "m")
        self.scrollAreaFrame.layout().addWidget(self.maxServiceDistanceSlider, 24, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        ## base time requesting
        ### label
        self.requestingServiceRequestingBaseTimeLabel = QLabel("Base Time (req.)")
        self.requestingServiceRequestingBaseTimeLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.requestingServiceRequestingBaseTimeLabel, 25, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.requestingServiceRequestingBaseTimeSlider = ArenaSliderWidget(0, 100, 1, "s")
        self.scrollAreaFrame.layout().addWidget(self.requestingServiceRequestingBaseTimeSlider, 25, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        ## base time receiving
        ### label
        self.requestingServiceReceivingBaseTimeLabel = QLabel("Base Time (recv.)")
        self.requestingServiceReceivingBaseTimeLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.requestingServiceReceivingBaseTimeLabel, 26, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.requestingServiceReceivingBaseTimeSlider = ArenaSliderWidget(0, 100, 1, "s")
        self.scrollAreaFrame.layout().addWidget(self.requestingServiceReceivingBaseTimeSlider, 26, 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # requesting guide
        ## heading
        self.requestingGuideLabel = QLabel("#### Requesting Guide")
        self.requestingGuideLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.requestingGuideLabel, 27, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        self.requestingGuideLine = Line()
        self.scrollAreaFrame.layout().addWidget(self.requestingGuideLine, 27, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        ## probability
        ### label
        self.requestingGuideProbabilityLabel = QLabel("Probability")
        self.requestingGuideProbabilityLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.requestingGuideProbabilityLabel, 28, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.requestingGuideProbabilitySlider = ArenaProbabilitySliderWidget()
        self.scrollAreaFrame.layout().addWidget(self.requestingGuideProbabilitySlider, 28, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        
        # requesting follower
        ## heading
        self.requestingFollowerLabel = QLabel("#### Requesting Follower")
        self.requestingFollowerLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.requestingFollowerLabel, 29, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        self.requestingFollowerLine = Line()
        self.scrollAreaFrame.layout().addWidget(self.requestingFollowerLine, 29, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        ## probability
        ### label
        self.requestingFollowerProbabilityLabel = QLabel("Probability")
        self.requestingFollowerProbabilityLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.requestingFollowerProbabilityLabel, 30, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.requestingFollowerProbabilitySlider = ArenaProbabilitySliderWidget()
        self.scrollAreaFrame.layout().addWidget(self.requestingFollowerProbabilitySlider, 30, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        
        # save button
        self.save_button = QtWidgets.QPushButton("Save and Close")
        self.save_button.clicked.connect(self.onSaveClicked)
        self.layout().addWidget(self.save_button, 1, 0, -1, -1)

    def updateWidgetsFromSelectedType(self):
        agent_type = PedsimAgentType(self.type_dropdown.currentIndex())
        if agent_type in [PedsimAgentType.ADULT, PedsimAgentType.CHILD, PedsimAgentType.ELDER]:
            # max talking distance
            self.maxTalkingDistanceLabel.show()
            self.maxTalkingDistanceSlider.show()
            # individual talking
            self.individualTalkingLabel.show()
            self.individualTalkingLine.show()
            self.individualTalkingProbabilityLabel.show()
            self.individualTalkingProbabilitySlider.show()
            self.individualTalkingBaseTimeLabel.show()
            self.individualTalkingBaseTimeSlider.show()
            # group talk
            self.groupTalkLabel.show()
            self.groupTalkLine.show()
            self.groupTalkProbabilityLabel.show()
            self.groupTalkProbabilitySlider.show()
            self.groupTalkBaseTimeLabel.show()
            self.groupTalkBaseTimeSlider.show()
            # talk and walk
            self.talkWalkLabel.show()
            self.talkWalkLine.show()
            self.talkWalkProbabilityLabel.show()
            self.talkWalkProbabilitySlider.show()
            self.talkWalkBaseTimeLabel.show()
            self.talkWalkBaseTimeSlider.show()
            # requesting service
            self.requestingServiceLabel.show()
            self.requestingServiceLine.show()
            self.requestingServiceProbabilityLabel.show()
            self.requestingServiceProbabilitySlider.show()
            self.maxServiceDistanceLabel.hide()
            self.maxServiceDistanceSlider.hide()
            self.requestingServiceRequestingBaseTimeLabel.show()
            self.requestingServiceRequestingBaseTimeSlider.show()
            self.requestingServiceReceivingBaseTimeLabel.show()
            self.requestingServiceReceivingBaseTimeSlider.show()
            # requesting guide
            self.requestingGuideLabel.show()
            self.requestingGuideLine.show()
            self.requestingGuideProbabilityLabel.show()
            self.requestingGuideProbabilitySlider.show()
            # requesting follower
            self.requestingFollowerLabel.show()
            self.requestingFollowerLine.show()
            self.requestingFollowerProbabilityLabel.show()
            self.requestingFollowerProbabilitySlider.show()

        elif agent_type == PedsimAgentType.VEHICLE:
            # max talking distance
            self.maxTalkingDistanceLabel.hide()
            self.maxTalkingDistanceSlider.hide()
            # individual talking
            self.individualTalkingLabel.hide()
            self.individualTalkingLine.hide()
            self.individualTalkingProbabilityLabel.hide()
            self.individualTalkingProbabilitySlider.hide()
            self.individualTalkingBaseTimeLabel.hide()
            self.individualTalkingBaseTimeSlider.hide()
            # group talk
            self.groupTalkLabel.hide()
            self.groupTalkLine.hide()
            self.groupTalkProbabilityLabel.hide()
            self.groupTalkProbabilitySlider.hide()
            self.groupTalkBaseTimeLabel.hide()
            self.groupTalkBaseTimeSlider.hide()
            # talk and walk
            self.talkWalkLabel.hide()
            self.talkWalkLine.hide()
            self.talkWalkProbabilityLabel.hide()
            self.talkWalkProbabilitySlider.hide()
            self.talkWalkBaseTimeLabel.hide()
            self.talkWalkBaseTimeSlider.hide()
            # requesting service
            self.requestingServiceLabel.hide()
            self.requestingServiceLine.hide()
            self.requestingServiceProbabilityLabel.hide()
            self.requestingServiceProbabilitySlider.hide()
            self.maxServiceDistanceLabel.hide()
            self.maxServiceDistanceSlider.hide()
            self.requestingServiceRequestingBaseTimeLabel.hide()
            self.requestingServiceRequestingBaseTimeSlider.hide()
            self.requestingServiceReceivingBaseTimeLabel.hide()
            self.requestingServiceReceivingBaseTimeSlider.hide()
            # requesting guide
            self.requestingGuideLabel.hide()
            self.requestingGuideLine.hide()
            self.requestingGuideProbabilityLabel.hide()
            self.requestingGuideProbabilitySlider.hide()
            # requesting follower
            self.requestingFollowerLabel.hide()
            self.requestingFollowerLine.hide()
            self.requestingFollowerProbabilityLabel.hide()
            self.requestingFollowerProbabilitySlider.hide()

        elif agent_type == PedsimAgentType.SERVICEROBOT:
            # max talking distance
            self.maxTalkingDistanceLabel.hide()
            self.maxTalkingDistanceSlider.hide()
            # individual talking
            self.individualTalkingLabel.hide()
            self.individualTalkingLine.hide()
            self.individualTalkingProbabilityLabel.hide()
            self.individualTalkingProbabilitySlider.hide()
            self.individualTalkingBaseTimeLabel.hide()
            self.individualTalkingBaseTimeSlider.hide()
            # group talk
            self.groupTalkLabel.hide()
            self.groupTalkLine.hide()
            self.groupTalkProbabilityLabel.hide()
            self.groupTalkProbabilitySlider.hide()
            self.groupTalkBaseTimeLabel.hide()
            self.groupTalkBaseTimeSlider.hide()
            # talk and walk
            self.talkWalkLabel.hide()
            self.talkWalkLine.hide()
            self.talkWalkProbabilityLabel.hide()
            self.talkWalkProbabilitySlider.hide()
            self.talkWalkBaseTimeLabel.hide()
            self.talkWalkBaseTimeSlider.hide()
            # requesting service
            self.requestingServiceLabel.show()
            self.requestingServiceLine.show()
            self.requestingServiceProbabilityLabel.hide()
            self.requestingServiceProbabilitySlider.hide()
            self.maxServiceDistanceLabel.show()
            self.maxServiceDistanceSlider.show()
            self.requestingServiceRequestingBaseTimeLabel.hide()
            self.requestingServiceRequestingBaseTimeSlider.hide()
            self.requestingServiceReceivingBaseTimeLabel.hide()
            self.requestingServiceReceivingBaseTimeSlider.hide()
            # requesting guide
            self.requestingGuideLabel.hide()
            self.requestingGuideLine.hide()
            self.requestingGuideProbabilityLabel.hide()
            self.requestingGuideProbabilitySlider.hide()
            # requesting follower
            self.requestingFollowerLabel.hide()
            self.requestingFollowerLine.hide()
            self.requestingFollowerProbabilityLabel.hide()
            self.requestingFollowerProbabilitySlider.hide()

    def updateValuesFromPedsimAgent(self):
        self.tempFlatlandModel.load(self.pedsimAgent.yaml_file)
        # TODO update all other values

    def onModelButtonClicked(self):
        rospack = rospkg.RosPack()
        initial_folder = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles")
        res = QtWidgets.QFileDialog.getOpenFileName(parent=self, directory=initial_folder)
        path = res[0]
        if path != "":
            self.setModelPath(path)

    def setModelPath(self, path: str):
        self.tempFlatlandModel.load(path)
        self.modelButton.setText(path.split("/")[-1])

    def onSaveClicked(self):
        self.save()
        self.pedsimAgentWidget.update()
        self.hide()

    def save(self):
        self.pedsimAgent.name = self.name_edit.text()
        self.pedsimAgent.yaml_file = self.tempFlatlandModel.path
        self.pedsimAgent.flatlandModel = self.tempFlatlandModel
        # TODO save data in widgets into pedsim agent
