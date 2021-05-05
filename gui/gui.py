from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPalette , QIcon ,QPixmap ,QFont
from functools import partial
import os, sys
import rospy
import rosnode
import rospkg
import json
import yaml
import numpy as np


class SubWindow(QWidget):
    def __init__(self):
        super().__init__()




class MainWindow(QMainWindow):
    
    def _read_stages_from_yaml(self):
        dir = rospkg.RosPack().get_path('arena_local_planner_drl')
        
        file_location = os.path.join( dir, 'configs', 'training_curriculum.yaml')
        print(file_location)
        if os.path.isfile(file_location):
            with open(file_location, "r") as file:
                self._stages = yaml.load(file, Loader=yaml.FullLoader)
            assert isinstance(
                self._stages, dict), "'training_curriculum.yaml' has wrong fromat! Has to encode dictionary!"
        else:
            raise FileNotFoundError(
                "Couldn't find 'training_curriculum.yaml' in %s " % self._PATHS.get('curriculum'))

    def _read_available_models_from_yaml(self):
        dir = rospkg.RosPack().get_path('arena_local_planner_drl')

        file_location = os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'available_models.yaml')
        
        if os.path.isfile(file_location):
            with open(file_location, "r") as file:
                self.available_models = yaml.load(file, Loader=yaml.FullLoader)
            assert isinstance(
                self.available_models, dict), "'available_models.yaml' has wrong fromat! Has to encode dictionary!"
        else:
            raise FileNotFoundError(
                "Couldn't find 'available_models.yaml' in %s " % file_location)

    def clear_layout(self,layout):

        for i in reversed(range(layout.count())): 
            layout.itemAt(i).widget().setParent(None)



    def on_button1_clicked(self, checked):

        ### keep the main window alive with the if statment ###
        if self.w is None:

            ### get current curriculum setting ###
            self._stages = dict()
            self._read_stages_from_yaml()
            self.num_stages = len( self._stages)
            
            
            ### get current available_models setting ###
            self.available_models = dict()
            self._read_available_models_from_yaml()
            print(self.available_models)
            
            ### call a sub window and label it ###
            self.window1 = SubWindow()
            self.window1.resize(450,350)
            self.window1.setWindowTitle('Training Curriculum Configs')
            self.num_stages_label = QLabel('Current num stages : ')
            self.num_stages_edit = QSpinBox()
            self.num_stages_edit.setValue(self.num_stages)
            self.num_stages_edit.textChanged.connect(self.update_window1)

            ### add the layout of the first window ###
            self.grid1 = QGridLayout()
            self.grid1.setSpacing(10)
            self.grid1.addWidget(self.num_stages_label , 1, 0)
            self.grid1.addWidget(self.num_stages_edit, 1, 1)
            self.window1.setLayout(self.grid1)
            self.update_window1(checked)
            self.window1.show()

        else:
            self.w.close()  # Close window.
            self.w = None  # Discard reference.

    def update_window1(self, checked):
        ### only intiate layout reseting when number is valid otherwise print error massage
        if self.num_stages_edit.text() != '' :
            if int(self.num_stages_edit.text())> 20 or int(self.num_stages_edit.text())  <1 :
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Critical)
                msg.setText("input invalid")
                msg.setInformativeText('please choose a number between 1 and 20')
                msg.setWindowTitle("Error")
                msg.exec_()
            
            else :
            ### reset all widgets out of window 1 ###
                self.clear_layout(self.grid1)
                myFont=QFont()
                myFont.setBold(True)
                label = QLabel('current num stages : ')
                label.setFont(myFont)
                self.grid1.addWidget(label, 1, 0)
                self.grid1.addWidget(self.num_stages_edit, 1, 1)

                ### add a group box to seperate from num stages box and some notation in Bold ###
                self.group_box_obstacles = QGroupBox("")
                grid = QGridLayout()
                grid.setSpacing(10)
                group_box_obstacles_labels = [QLabel('stage number'),QLabel('static obstacles'),QLabel('human obstacles'),QLabel('robot obstacles')]
                for i in range(len(group_box_obstacles_labels)) :
                    group_box_obstacles_labels[i].setFont(myFont)
                    grid.addWidget(group_box_obstacles_labels[i], 1, i)
                self.group_box_obstacles.setLayout(grid)
              
                ### generate for every stage variable entry boxes and add default values to them ###
                stages_values = list(self._stages.values())
                for i in range(int(self.num_stages_edit.text())) :         
                    new_widgets = [QPushButton('stage num '+ str(i+1)),QSpinBox(),QSpinBox(),QSpinBox()]
                    new_widgets[0].clicked.connect(partial(self.on_button_stage_clicked, checked ,'Stage '+ str(i+1)))
                    new_widgets[1].setValue(0)
                    new_widgets[2].setValue(5)
                    new_widgets[3].setValue(2)
                    if i < len(stages_values)  and stages_values[i] is not None :
                        new_widgets[1].setValue(stages_values[i]['static'])
                        new_widgets[2].setValue(stages_values[i]['dynamic_human'])
                        new_widgets[3].setValue(stages_values[i]['dynamic_robot'])
                    for j in range(4):
                        grid.addWidget(new_widgets[j], i+2, j)
                self.grid1.addWidget(self.group_box_obstacles,2,0)
                self.window1.show()

    def on_button_stage_clicked(self, checked, button_name):

        if not (self.window1  is  None):

            ### get current curriculum setting ###
            ### call a sub window and label it ###
            self.window_extended_stage= SubWindow()
            self.window_extended_stage.resize(450,350)
            self.window_extended_stage.setWindowTitle(button_name + ' extended configs')
        
            # self.group_box_static obstacles = QGroupBox("Human obstacles")
            # self.group_box_obstacles = QGroupBox("Human obstacles")


            #     grid = QGridLayout()
            #     grid.setSpacing(10)
            #     group_box_obstacles_labels = [QLabel('stage number'),QLabel('static obstacles'),QLabel('human obstacles'),QLabel('robot obstacles')]
            #     for i in range(len(group_box_obstacles_labels)) :
            #         group_box_obstacles_labels[i].setFont(myFont)
            #         grid.addWidget(group_box_obstacles_labels[i], 1, i)
            
            #     self.group_box_obstacles.setLayout(grid)

            # ### add the layout of the first window ###
            # self.grid1 = QGridLayout()
            # self.grid1.setSpacing(10)
            # self.grid1.addWidget(self.num_stages_label , 1, 0)
            # self.grid1.addWidget(self.num_stages_edit, 1, 1)
            # self.window1.setLayout(self.grid1)
            self.window_extended_stage.show()

        else:
            self.window1 .close()  # Close window.
            self.window1 = None  # Discard reference.

    def on_button2_clicked(self, checked):

        if self.w is None:
            self.window2 = SubWindow()
            self.window2.setWindowTitle('Obstacles Profiles Configs')
            self.window2.show()
        else:
            self.w.close()  # Close window.
            self.w = None  # Discard reference.

    def on_button3_clicked(self, checked):
        if self.w is None:
            self.window3 = SubWindow()
            self.window3.setWindowTitle('New Model Configs')

            self.window3.show()
        else:
            self.w.close()  # Close window.
            self.w = None  # Discard reference.


    def __init__(self):
        super().__init__()

        self.w = None  
        self.window = QWidget(self)
        ### add label, size and center the main window ###
        self.setCentralWidget(self.window)
        self.setWindowTitle('Arena Configs')
        self.resize(350, 150)
        icon = QIcon()
        icon.addPixmap(QPixmap('icon.png'), QIcon.Selected, QIcon.On)
        self.setWindowIcon(icon)
        qtRectangle = self.window.frameGeometry()
        centerPoint = QDesktopWidget().availableGeometry().center()
        qtRectangle.moveCenter(centerPoint)
        self.move(qtRectangle.topLeft())
        self.window.setWindowIcon(icon)
        
        ### add diffrent buttons to the main window and link them to functions ###
        self.button1 = QPushButton('Edit Training Curriculum')
        self.button1.clicked.connect(self.on_button1_clicked)
        self.button2 = QPushButton('Edit Obstacles Profiles')
        self.button2.clicked.connect(self.on_button2_clicked)
        self.button3 = QPushButton('Create New Model')
        self.button3.clicked.connect(self.on_button3_clicked)
       
        ### add the layout of the main window ###
        layout = QVBoxLayout()
        layout.addWidget(self.button1)
        layout.addWidget(self.button2)
        layout.addWidget(self.button3)
        self.window.setLayout(layout)

       

        

app = QApplication([])
app.setStyle('Fusion')
w = MainWindow()
w.show()

app.exec()



