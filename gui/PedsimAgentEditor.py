from typing import Any
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtCore import Qt, QRect
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QGridLayout, QVBoxLayout, QFrame, QGraphicsScene, QGraphicsView, QScrollArea, QHBoxLayout, QPushButton, QSpacerItem, QSizePolicy, QLabel, QLineEdit, QComboBox, QSpinBox, QSlider
from geometry_msgs.msg import Point
import rospkg
import os
import copy
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
        self.updateWidgetsFromSelectedStartupMode()

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

        vertical_idx = 0

        # heading "general"
        general_label = QLabel("#### General")
        general_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(general_label, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        line = Line()
        self.scrollAreaFrame.layout().addWidget(line, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1

        # name
        ## label
        name_label = QLabel("Name")
        name_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(name_label, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## editbox
        self.name_edit = QLineEdit(self.pedsimAgentWidget.name_label.text())
        self.name_edit.setFixedSize(200, 30)
        self.scrollAreaFrame.layout().addWidget(self.name_edit, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1

        # model file
        ## label
        name_label = QLabel("Flatland Model")
        name_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(name_label, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## choose button
        self.modelButton = QPushButton("Choose...")
        self.modelButton.setFixedSize(200, 30)
        self.modelButton.clicked.connect(self.onModelButtonClicked)
        self.scrollAreaFrame.layout().addWidget(self.modelButton, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1

        # type
        ## label
        type_label = QLabel("Type")
        type_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(type_label, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## dropdown
        self.typeComboBox = QComboBox()
        for agent_type in PedsimAgentType:
            self.typeComboBox.insertItem(agent_type.value, agent_type.name.lower())
        self.typeComboBox.setFixedSize(200, 30)
        self.typeComboBox.currentIndexChanged.connect(self.updateWidgetsFromSelectedType)
        self.scrollAreaFrame.layout().addWidget(self.typeComboBox, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1

        # amount
        ## label
        amount_label = QLabel("Amount")
        amount_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(amount_label, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## spin box
        self.amountSpinBox = QSpinBox()
        self.amountSpinBox.setValue(1)
        self.amountSpinBox.setMinimum(1)
        self.amountSpinBox.setFixedSize(200, 30)
        self.scrollAreaFrame.layout().addWidget(self.amountSpinBox, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1

        # waypoint mode
        ## label
        label = QLabel("Waypoint Mode")
        label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(label, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## dropdown
        self.waypointModeComboBox = QComboBox()
        for mode in PedsimWaypointMode:
            self.waypointModeComboBox.insertItem(mode.value, mode.name.lower())
        self.waypointModeComboBox.setFixedSize(200, 30)
        self.scrollAreaFrame.layout().addWidget(self.waypointModeComboBox, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1

        # startup mode
        ## label
        label = QLabel("Startup Mode")
        label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(label, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## dropdown
        self.startupModeComboBox = QComboBox()
        for mode in PedsimStartupMode:
            self.startupModeComboBox.insertItem(mode.value, mode.name.lower())
        self.startupModeComboBox.setFixedSize(200, 30)
        self.startupModeComboBox.currentIndexChanged.connect(self.updateWidgetsFromSelectedStartupMode)
        self.scrollAreaFrame.layout().addWidget(self.startupModeComboBox, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1

        # wait time
        ## label
        self.waitTimeLabel = QLabel("Wait Time")
        self.waitTimeLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.waitTimeLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## spin box
        self.waitTimeSpinBox = ArenaQDoubleSpinBox()
        self.waitTimeSpinBox.setFixedSize(200, 30)
        self.scrollAreaFrame.layout().addWidget(self.waitTimeSpinBox, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1

        # trigger zone radius
        ## label
        self.triggerZoneLabel = QLabel("Trigger Zone Radius")
        self.triggerZoneLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.triggerZoneLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## spin box
        self.triggerZoneSpinBox = ArenaQDoubleSpinBox()
        self.triggerZoneSpinBox.setFixedSize(200, 30)
        self.scrollAreaFrame.layout().addWidget(self.triggerZoneSpinBox, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1

        # vmax
        ## label
        self.vmax_label = QLabel("Velocity<sub>max</sub>")
        self.vmax_label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.vmax_label, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ## slider
        self.vmax_slider = ArenaSliderWidget(0, 20, 0.1, "m/s")
        self.scrollAreaFrame.layout().addWidget(self.vmax_slider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1

        # max talking distance
        ### label
        self.maxTalkingDistanceLabel = QLabel("Max Talking Distance")
        self.maxTalkingDistanceLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.maxTalkingDistanceLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.maxTalkingDistanceSlider = ArenaSliderWidget(0, 10, 1, "m")
        self.scrollAreaFrame.layout().addWidget(self.maxTalkingDistanceSlider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1

        # forces
        ## heading
        label = QLabel("#### Force Factors")
        label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(label, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        line = Line()
        self.scrollAreaFrame.layout().addWidget(line, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        ## Desired
        ### label
        label = QLabel("Desired")
        label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(label, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.desiredForceSlider = ArenaSliderWidget(0, 20, 1, "")
        self.desiredForceSlider.slider.setValue(1)
        self.scrollAreaFrame.layout().addWidget(self.desiredForceSlider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        ## Obstacle
        ### label
        label = QLabel("Obstacle")
        label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(label, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.obstacleForceSlider = ArenaSliderWidget(0, 20, 1, "")
        self.obstacleForceSlider.slider.setValue(1)
        self.scrollAreaFrame.layout().addWidget(self.obstacleForceSlider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        ## Social
        ### label
        label = QLabel("Social")
        label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(label, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.socialForceSlider = ArenaSliderWidget(0, 20, 1, "")
        self.socialForceSlider.slider.setValue(1)
        self.scrollAreaFrame.layout().addWidget(self.socialForceSlider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        ## Robot
        ### label
        label = QLabel("Robot")
        label.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(label, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.robotForceSlider = ArenaSliderWidget(0, 20, 1, "")
        self.robotForceSlider.slider.setValue(1)
        self.scrollAreaFrame.layout().addWidget(self.robotForceSlider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1


        # individual talking
        ## heading
        self.individualTalkingLabel = QLabel("#### Individual Talking")
        self.individualTalkingLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.individualTalkingLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        self.individualTalkingLine = Line()
        self.scrollAreaFrame.layout().addWidget(self.individualTalkingLine, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        ## probability
        ### label
        self.individualTalkingProbabilityLabel = QLabel("Probability")
        self.individualTalkingProbabilityLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.individualTalkingProbabilityLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.individualTalkingProbabilitySlider = ArenaProbabilitySliderWidget()
        self.scrollAreaFrame.layout().addWidget(self.individualTalkingProbabilitySlider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        ## base time
        ### label
        self.individualTalkingBaseTimeLabel = QLabel("Base Time")
        self.individualTalkingBaseTimeLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.individualTalkingBaseTimeLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.individualTalkingBaseTimeSlider = ArenaSliderWidget(0, 100, 1, "s")
        self.scrollAreaFrame.layout().addWidget(self.individualTalkingBaseTimeSlider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1

        # group talk
        ## heading
        self.groupTalkLabel = QLabel("#### Group Talk")
        self.groupTalkLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.groupTalkLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        self.groupTalkLine = Line()
        self.scrollAreaFrame.layout().addWidget(self.groupTalkLine, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        ## probability
        ### label
        self.groupTalkProbabilityLabel = QLabel("Probability")
        self.groupTalkProbabilityLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.groupTalkProbabilityLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.groupTalkProbabilitySlider = ArenaProbabilitySliderWidget()
        self.scrollAreaFrame.layout().addWidget(self.groupTalkProbabilitySlider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        ## base time
        ### label
        self.groupTalkBaseTimeLabel = QLabel("Base Time")
        self.groupTalkBaseTimeLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.groupTalkBaseTimeLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.groupTalkBaseTimeSlider = ArenaSliderWidget(0, 100, 1, "s")
        self.scrollAreaFrame.layout().addWidget(self.groupTalkBaseTimeSlider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1

        # talk and walk
        ## heading
        self.talkWalkLabel = QLabel("#### Talk & Walk")
        self.talkWalkLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.talkWalkLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        self.talkWalkLine = Line()
        self.scrollAreaFrame.layout().addWidget(self.talkWalkLine, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        ## probability
        ### label
        self.talkWalkProbabilityLabel = QLabel("Probability")
        self.talkWalkProbabilityLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.talkWalkProbabilityLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.talkWalkProbabilitySlider = ArenaProbabilitySliderWidget()
        self.scrollAreaFrame.layout().addWidget(self.talkWalkProbabilitySlider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        ## base time
        ### label
        self.talkWalkBaseTimeLabel = QLabel("Base Time")
        self.talkWalkBaseTimeLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.talkWalkBaseTimeLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.talkWalkBaseTimeSlider = ArenaSliderWidget(0, 100, 1, "s")
        self.scrollAreaFrame.layout().addWidget(self.talkWalkBaseTimeSlider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1

        # requesting service
        ## heading
        self.requestingServiceLabel = QLabel("#### Requesting Service")
        self.requestingServiceLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.requestingServiceLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        self.requestingServiceLine = Line()
        self.scrollAreaFrame.layout().addWidget(self.requestingServiceLine, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        ## probability
        ### label
        self.requestingServiceProbabilityLabel = QLabel("Probability")
        self.requestingServiceProbabilityLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.requestingServiceProbabilityLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.requestingServiceProbabilitySlider = ArenaProbabilitySliderWidget()
        self.scrollAreaFrame.layout().addWidget(self.requestingServiceProbabilitySlider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        ## max service distance
        ### label
        self.maxServiceDistanceLabel = QLabel("Max Service Distance")
        self.maxServiceDistanceLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.maxServiceDistanceLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.maxServiceDistanceSlider = ArenaSliderWidget(0, 10, 1, "m")
        self.scrollAreaFrame.layout().addWidget(self.maxServiceDistanceSlider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        ## base time requesting
        ### label
        self.requestingServiceRequestingBaseTimeLabel = QLabel("Base Time (req.)")
        self.requestingServiceRequestingBaseTimeLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.requestingServiceRequestingBaseTimeLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.requestingServiceRequestingBaseTimeSlider = ArenaSliderWidget(0, 100, 1, "s")
        self.scrollAreaFrame.layout().addWidget(self.requestingServiceRequestingBaseTimeSlider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        ## base time receiving
        ### label
        self.requestingServiceReceivingBaseTimeLabel = QLabel("Base Time (recv.)")
        self.requestingServiceReceivingBaseTimeLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.requestingServiceReceivingBaseTimeLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.requestingServiceReceivingBaseTimeSlider = ArenaSliderWidget(0, 100, 1, "s")
        self.scrollAreaFrame.layout().addWidget(self.requestingServiceReceivingBaseTimeSlider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1

        # requesting guide
        ## heading
        self.requestingGuideLabel = QLabel("#### Requesting Guide")
        self.requestingGuideLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.requestingGuideLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        self.requestingGuideLine = Line()
        self.scrollAreaFrame.layout().addWidget(self.requestingGuideLine, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        ## probability
        ### label
        self.requestingGuideProbabilityLabel = QLabel("Probability")
        self.requestingGuideProbabilityLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.requestingGuideProbabilityLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.requestingGuideProbabilitySlider = ArenaProbabilitySliderWidget()
        self.scrollAreaFrame.layout().addWidget(self.requestingGuideProbabilitySlider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        
        # requesting follower
        ## heading
        self.requestingFollowerLabel = QLabel("#### Requesting Follower")
        self.requestingFollowerLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.requestingFollowerLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        self.requestingFollowerLine = Line()
        self.scrollAreaFrame.layout().addWidget(self.requestingFollowerLine, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        ## probability
        ### label
        self.requestingFollowerProbabilityLabel = QLabel("Probability")
        self.requestingFollowerProbabilityLabel.setTextFormat(QtCore.Qt.TextFormat.MarkdownText)
        self.scrollAreaFrame.layout().addWidget(self.requestingFollowerProbabilityLabel, vertical_idx, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        ### slider
        self.requestingFollowerProbabilitySlider = ArenaProbabilitySliderWidget()
        self.scrollAreaFrame.layout().addWidget(self.requestingFollowerProbabilitySlider, vertical_idx, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        vertical_idx += 1
        
        # save button
        self.save_button = QtWidgets.QPushButton("Save and Close")
        self.save_button.clicked.connect(self.onSaveClicked)
        self.layout().addWidget(self.save_button, 1, 0, -1, -1)

    def updateWidgetsFromSelectedStartupMode(self):
        mode = PedsimStartupMode(self.startupModeComboBox.currentIndex())
        if mode == PedsimStartupMode.DEFAULT:
            self.waitTimeLabel.hide()
            self.waitTimeSpinBox.hide()
            self.triggerZoneLabel.hide()
            self.triggerZoneSpinBox.hide()
        elif mode == PedsimStartupMode.WAITTIMER:
            self.waitTimeLabel.show()
            self.waitTimeSpinBox.show()
            self.triggerZoneLabel.hide()
            self.triggerZoneSpinBox.hide()
        elif mode == PedsimStartupMode.TRIGGERZONE:
            self.waitTimeLabel.hide()
            self.waitTimeSpinBox.hide()
            self.triggerZoneLabel.show()
            self.triggerZoneSpinBox.show()

    def updateWidgetsFromSelectedType(self):
        agent_type = PedsimAgentType(self.typeComboBox.currentIndex())
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
        self.setModelPath(self.pedsimAgent.yaml_file)

        self.typeComboBox.setCurrentIndex(PedsimAgentType[self.pedsimAgent.type.upper()].value)
        self.amountSpinBox.setValue(self.pedsimAgent.number_of_peds)
        self.vmax_slider.setValue(self.pedsimAgent.vmax)

        self.startupModeComboBox.setCurrentIndex(PedsimStartupMode[self.pedsimAgent.start_up_mode.upper()].value)
        self.waitTimeSpinBox.setValue(self.pedsimAgent.wait_time)
        self.triggerZoneSpinBox.setValue(self.pedsimAgent.trigger_zone_radius)

        self.individualTalkingProbabilitySlider.setValue(self.pedsimAgent.chatting_probability)
        self.groupTalkProbabilitySlider.setValue(self.pedsimAgent.group_talking_probability)
        self.talkWalkProbabilitySlider.setValue(self.pedsimAgent.talking_and_walking_probability)
        self.requestingServiceProbabilitySlider.setValue(self.pedsimAgent.requesting_service_probability)
        self.requestingGuideProbabilitySlider.setValue(self.pedsimAgent.requesting_guide_probability)
        self.requestingFollowerProbabilitySlider.setValue(self.pedsimAgent.requesting_follower_probability)

        self.maxTalkingDistanceSlider.setValue(self.pedsimAgent.max_talking_distance)
        self.maxServiceDistanceSlider.setValue(self.pedsimAgent.max_servicing_radius)

        self.individualTalkingBaseTimeSlider.setValue(self.pedsimAgent.talking_base_time)
        self.groupTalkBaseTimeSlider.setValue(self.pedsimAgent.group_talking_base_time)
        self.talkWalkBaseTimeSlider.setValue(self.pedsimAgent.talking_and_walking_base_time)
        self.requestingServiceReceivingBaseTimeSlider.setValue(self.pedsimAgent.receiving_service_base_time)
        self.requestingServiceRequestingBaseTimeSlider.setValue(self.pedsimAgent.requesting_service_base_time)

        # forces
        self.desiredForceSlider.setValue(self.pedsimAgent.force_factor_desired)
        self.obstacleForceSlider.setValue(self.pedsimAgent.force_factor_obstacle)
        self.socialForceSlider.setValue(self.pedsimAgent.force_factor_social)
        self.robotForceSlider.setValue(self.pedsimAgent.force_factor_robot)

        self.waypointModeComboBox.setCurrentIndex(PedsimWaypointMode(self.pedsimAgent.waypoint_mode).value)

        self.name_edit.setText(self.pedsimAgent.name)

    def show(self):
        self.updateValuesFromPedsimAgent()
        return super().show()

    def onModelButtonClicked(self):
        rospack = rospkg.RosPack()
        initial_folder = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles")
        res = QtWidgets.QFileDialog.getOpenFileName(parent=self, directory=initial_folder)
        path = res[0]
        if path != "":
            self.setModelPath(path)

    def setModelPath(self, path: str):
        if os.path.exists(path):
            self.tempFlatlandModel.load(path)
            self.modelButton.setText(path.split("/")[-1])

    def getPedsimAgentFromWidgets(self) -> PedsimAgent:
        agent = copy.deepcopy(self.pedsimAgent)
        agent.type = PedsimAgentType(self.typeComboBox.currentIndex()).name.lower()
        agent.number_of_peds = self.amountSpinBox.value()
        agent.vmax = self.vmax_slider.getValue()

        agent.start_up_mode = PedsimStartupMode(self.startupModeComboBox.currentIndex()).name.lower()
        agent.wait_time = self.waitTimeSpinBox.value()
        agent.trigger_zone_radius = self.triggerZoneSpinBox.value()

        agent.chatting_probability = self.individualTalkingProbabilitySlider.getValue()
        agent.group_talking_probability = self.groupTalkProbabilitySlider.getValue()
        agent.talking_and_walking_probability = self.talkWalkProbabilitySlider.getValue()
        agent.requesting_service_probability = self.requestingServiceProbabilitySlider.getValue()
        agent.requesting_guide_probability = self.requestingGuideProbabilitySlider.getValue()
        agent.requesting_follower_probability = self.requestingFollowerProbabilitySlider.getValue()

        agent.max_talking_distance = self.maxTalkingDistanceSlider.getValue()
        agent.max_servicing_radius = self.maxServiceDistanceSlider.getValue()

        agent.talking_base_time = self.individualTalkingBaseTimeSlider.getValue()
        agent.group_talking_base_time = self.groupTalkBaseTimeSlider.getValue()
        agent.talking_and_walking_base_time = self.talkWalkBaseTimeSlider.getValue()
        agent.receiving_service_base_time = self.requestingServiceReceivingBaseTimeSlider.getValue()
        agent.requesting_service_base_time = self.requestingServiceRequestingBaseTimeSlider.getValue()

        # forces
        agent.force_factor_desired = self.desiredForceSlider.getValue()
        agent.force_factor_obstacle = self.obstacleForceSlider.getValue()
        agent.force_factor_social = self.socialForceSlider.getValue()
        agent.force_factor_robot = self.robotForceSlider.getValue()

        agent.waypoint_mode = PedsimWaypointMode(self.waypointModeComboBox.currentIndex()).value

        agent.yaml_file = self.tempFlatlandModel.path

        agent.name = self.name_edit.text()
        agent.flatlandModel = self.tempFlatlandModel

        return agent

    def onSaveClicked(self):
        self.save()
        self.hide()

    def save(self):
        self.pedsimAgent = self.getPedsimAgentFromWidgets()
        self.pedsimAgentWidget.setPedsimAgent(self.pedsimAgent)
    
    def closeEvent(self, event):
        # check if any changes have been made
        current_agent = self.getPedsimAgentFromWidgets()
        if self.pedsimAgent != current_agent:
            # ask user if she wants to save changes
            msg_box = QtWidgets.QMessageBox()
            msg_box.setText("Do you want to save changes to this agent?")
            msg_box.setStandardButtons(QtWidgets.QMessageBox.Save | QtWidgets.QMessageBox.Discard | QtWidgets.QMessageBox.Cancel)
            msg_box.setDefaultButton(QtWidgets.QMessageBox.Save)
            ret = msg_box.exec()
            if ret == QtWidgets.QMessageBox.Save:
                self.onSaveClicked()
            elif ret == QtWidgets.QMessageBox.Discard:
                # reset to values already saved
                self.updateValuesFromPedsimAgent()
            elif ret == QtWidgets.QMessageBox.Cancel:
                event.ignore()

