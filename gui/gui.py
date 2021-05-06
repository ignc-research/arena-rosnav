from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt ,QMetaObject
from PyQt5.QtGui import QPalette , QIcon ,QPixmap ,QFont
from functools import partial
import os, sys
import rospy
import rosnode
import rospkg
import json
import yaml
import numpy as np



def read_stages_from_yaml():
    dir = rospkg.RosPack().get_path('arena_local_planner_drl')
    
    
    file_location = os.path.join( dir, 'configs', 'training_curriculum.yaml')
    
    if os.path.isfile(file_location):
        with open(file_location, "r") as file:
           w._stages = yaml.load(file, Loader=yaml.FullLoader)
        assert isinstance(
             w._stages, dict), "'training_curriculum.yaml' has wrong fromat! Has to encode dictionary!"
        
    else:
        raise FileNotFoundError(
            "Couldn't find 'training_curriculum.yaml' in %s " % file_location)

def write_stages_to_yaml(new_stages):
    dir = rospkg.RosPack().get_path('arena_local_planner_drl')
    
    file_location = os.path.join( dir, 'configs', 'training_curriculum.yaml')
  
    if os.path.isfile(file_location):
        with open(file_location, "w") as file:
            # file.seek(0) 
            
            # # to erase all data 
            # file.truncate() 
            yaml.dump(new_stages, file, default_flow_style=False, sort_keys=False)
        assert isinstance(
            new_stages, dict), "'training_curriculum.yaml' has wrong fromat! Has to encode dictionary!"
    else:
        raise FileNotFoundError(
            "Couldn't find 'training_curriculum.yaml' in %s " % file_location)


def read_available_models_from_yaml():
    dir = rospkg.RosPack().get_path('arena_local_planner_drl')

    global available_models
    file_location = os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'available_models.yaml')
    
    if os.path.isfile(file_location):
        with open(file_location, "r") as file:
            available_models = yaml.load(file, Loader=yaml.FullLoader)
        assert isinstance(
            available_models, dict), "'available_models.yaml' has wrong fromat! Has to encode dictionary!"
    else:
        raise FileNotFoundError(
            "Couldn't find 'available_models.yaml' in %s " % file_location)


def clear_layout(layout):

    for i in reversed(range(layout.count())): 
        layout.itemAt(i).widget().setParent(None)



class Window1(QWidget):
        
 

    def __init__(self):
        super().__init__()

        ### get current curriculum setting ###        
        read_stages_from_yaml()  
        ### get current available_models setting ###      
        read_available_models_from_yaml()

    
    def set_up_window1_ui(self): 
            self.resize(450,350)
            self.setWindowTitle('Training Curriculum Configs')
            self.num_stages_label = QLabel('Current num stages : ')
            self.num_stages_edit = QSpinBox()
            self.num_stages_edit.setValue(len( w._stages)  )
            self.num_stages_edit.textChanged.connect(self.update_window1)
            self.advanced_group_box = QGroupBox("       advanced configs", self)
            self.chBox = QCheckBox("Advanced", self.advanced_group_box)

            ### add the layout of the first window ###
            self.grid1 = QGridLayout()
            self.grid1.setSpacing(10)
            self.setLayout(self.grid1)
            self.update_window1()
            self.show()

    def update_window1(self):
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
                clear_layout(self.grid1)
                myFont=QFont()
                myFont.setBold(True)
                label = QLabel('current num stages : ')
                label.setFont(myFont)
                self.advanced_group_box.setFont(myFont)
                self.grid1.addWidget(label, 1, 0)
                self.grid1.addWidget(self.num_stages_edit, 1, 1)
                self.grid1.addWidget(self.chBox, 1, 2)
                self.grid1.addWidget(self.advanced_group_box, 2, 2)


                self.on_button_advanced_configs_clicked(self.chBox.checkState())
                self.chBox.stateChanged.connect(self.on_button_advanced_configs_clicked)

                ### add a group box to seperate from num stages box and some notation in Bold ###
                self.group_box_obstacles = QGroupBox("")
                grid = QGridLayout()
                grid.setSpacing(10)

                group_box_obstacles_labels = [QLabel('stage number'),QLabel('static obstacles'),QLabel('human obstacles'),QLabel('robot obstacles')]
                for i in range(len(group_box_obstacles_labels)) :
                    group_box_obstacles_labels[i].setFont(myFont)
                    grid.addWidget(group_box_obstacles_labels[i], 1, i,Qt.AlignTop)
                self.group_box_obstacles.setLayout(grid)
              
                ### generate for every stage variable entry boxes and add default values to them ###
                self.training_curriculum_widgets =[]
                stages_values = list(w._stages.values())
                for i in range(int(self.num_stages_edit.text())) :         
                    self.new_widgets = [QPushButton('stage num '+ str(i+1)),QSpinBox(),QSpinBox(),QSpinBox()]
                    self.new_widgets[0].clicked.connect(partial(self.on_button_stage_clicked,'Stage '+ str(i+1)))
                    self.new_widgets[1].setValue(0)
                    self.new_widgets[2].setValue(5)
                    self.new_widgets[3].setValue(2)
                    self.training_curriculum_widgets = self.training_curriculum_widgets +[self.new_widgets]
                    if i < len(stages_values)  and stages_values[i] is not None :
                        self.new_widgets[1].setValue(stages_values[i]['static'])
                        self.new_widgets[2].setValue(stages_values[i]['dynamic_human'])
                        self.new_widgets[3].setValue(stages_values[i]['dynamic_robot'])
                    for j in range(4):
                        grid.addWidget(self.new_widgets[j], i+2, j,Qt.AlignTop)
                self.grid1.addWidget(self.group_box_obstacles,2,0,2,2)


    def on_button_advanced_configs_clicked(self,s):
        if s == Qt.Checked:
            self.advanced_group_box.show()
        else: 
            self.advanced_group_box.hide()

    def on_button_stage_clicked(self, button_name):

        if not (self is  None):

            ### call a sub window and label it ###
            self.window_extended_stage= window_extended_stage()
            self.window_extended_stage.set_up_ui_window_extended_stage(button_name)
            self.window_extended_stage.show()

        else:
            self.close()  # Close window.
            self = None  # Discard reference.

    def closeEvent(self, event):
   

 
        reply = QMessageBox.question(self,
                                            'This program',
                                            "Do you want to save the configs of training curriculum?",
                                            QMessageBox.Yes | QMessageBox.No,
                                            QMessageBox.No)
        if reply == QMessageBox.Yes:
            new_stages ={}
            for i , training_curriculum_stage_widgets in enumerate(self.training_curriculum_widgets ) :
                new_stages[i+1]= {'dynamic_human':int(training_curriculum_stage_widgets[2].text()),'dynamic_robot':int(training_curriculum_stage_widgets[3].text()),'static':int(training_curriculum_stage_widgets[1].text())}
            write_stages_to_yaml(new_stages)
            

class window_extended_stage(QWidget):
    def __init__(self):
        super().__init__()

    def set_up_ui_window_extended_stage(self,stage_name): 
        self.setObjectName("window_extended_"+stage_name)
        self.setWindowTitle(stage_name + ' extended configs')    
        window_extended_stage_grid = QGridLayout()
        self.setLayout(window_extended_stage_grid)
    

        group_box_dict = {QGroupBox("        static obstacles"):[QGridLayout(),0], QGroupBox("        human obstacles"):[QGridLayout(),1],QGroupBox("        robot obstacles"):[QGridLayout(),2]}
        
        ### get available_models out of yaml Datei and write the values into the group boxes ###
        available_models_values = list( available_models.values())
        for box, layout in group_box_dict.items():
            box.setLayout(layout[0])
            
            if available_models_values[layout[1]] is not None:
                for i,available_model in enumerate(list( available_models_values[layout[1]].keys())):
                    layout[0].addWidget(QLabel(available_model),i,0,Qt.AlignTop)
                    layout[0].addWidget(QSpinBox(),i,1,Qt.AlignTop)

            window_extended_stage_grid.addWidget(box,0,layout[1],Qt.AlignTop)

    def closeEvent(self, event):
   

        reply = QMessageBox.question(self,
                                            'This program',
                                            'Do you want to save the configs of s'+ self.objectName()[17:]+'?',
                                            QMessageBox.Yes | QMessageBox.No,
                                            QMessageBox.No)
            # if reply == QMessageBox.Yes:


class main_window(QMainWindow):
    

    def __init__(self):
        super().__init__()

        self.w = None  
        self._stages = dict()
        self.available_models = dict()

    def set_up_main_window_ui(self): 
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
        self.show()

    def on_button1_clicked(self, checked):

        ### keep the main window alive with the if statment ###
        if self.w is None:
            ### call a sub window and label it ###
            self.window1 = Window1()
            self.window1.set_up_window1_ui()
        else:
            self.w.close()  # Close window.
            self.w = None  # Discard reference.

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
     

        

app = QApplication([])
app.setStyle('Fusion')
w = main_window()
w.set_up_main_window_ui()

app.exec()



