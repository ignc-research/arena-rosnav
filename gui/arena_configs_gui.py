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
import copy
from FlatlandModelEditor import FlatlandModelEditor


def get_value_qspin_box(x) :

    return x.value()
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
            yaml.dump(new_stages, file, default_flow_style=False, sort_keys=False)
        assert isinstance(
            new_stages, dict), "'training_curriculum.yaml' has wrong fromat! Has to encode dictionary!"
        file.close()
    else:
        raise FileNotFoundError(
            "Couldn't find 'training_curriculum.yaml' in %s " % file_location)


def read_available_models_from_yaml():
 
    file_location = os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'available_models.yaml')
    
    if os.path.isfile(file_location):
        with open(file_location, "r") as file:
            w.available_models = yaml.load(file, Loader=yaml.FullLoader)
        assert isinstance(
            w.available_models, dict), "'available_models.yaml' has wrong fromat! Has to encode dictionary!"
    else:
        raise FileNotFoundError(
            "Couldn't find 'available_models.yaml' in %s " % file_location)

def write_obstacles_spawning_parameters_to_yaml(obstacles_spawning_parameters):
    
    file_location = os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'obstacles_spawning_parameters.yaml')
    
    
    if os.path.isfile(file_location):
        with open(file_location, "w") as file:  
            yaml.dump(obstacles_spawning_parameters, file, default_flow_style=False, sort_keys=False)
        

    else:
        raise FileNotFoundError(
            "Couldn't find 'obstacles_spawning_parameters.yaml' in %s " % file_location)

def write_advanced_configs_to_yaml(advanced_configs):
    
    file_location = os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'advanced_configs.yaml')
    
    
    if os.path.isfile(file_location):
        with open(file_location, "w") as file:  
            yaml.dump(advanced_configs, file, default_flow_style=False, sort_keys=False)
        

    else:
        raise FileNotFoundError(
            "Couldn't find 'advanced_configs.yaml' in %s " % file_location)

def read_advanced_configs_from_yaml():
    
    file_location = os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'advanced_configs.yaml')
    
    
    if os.path.isfile(file_location):
        with open(file_location, "r") as file:
            w.advanced_configs = yaml.load(file, Loader=yaml.FullLoader)

    else:
        raise FileNotFoundError(
            "Couldn't find 'advanced_configs.yaml' in %s " % file_location)



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
        ### get current advanced_configs setting ###     
        read_advanced_configs_from_yaml()

    
    def set_up_window1_ui(self): 
            
            self.setWindowTitle('Training Curriculum Configs')
            self.num_stages_label = QLabel('Current num stages : ')
            self.num_stages_edit = QSpinBox()
            self.num_stages_edit.setValue(len( w._stages)  )
            self.num_stages_edit.textChanged.connect(self.update_window1)
            self.advanced_group_box = QGroupBox("", self)
            self.chBox = QCheckBox("Advanced", self.advanced_group_box)
            self.grid_advanced_group_box = QGridLayout()
            self.grid_advanced_group_box.setSpacing(10)
            self.advanced_group_box.setLayout(self.grid_advanced_group_box)

            ### add the layout of the first window ###
            self.grid1 = QGridLayout()
            self.grid1.setSpacing(10)
            self.setLayout(self.grid1)
            self.update_window1()
            self.show()

    def update_window1(self):
        ### only intiate layout reseting when number is valid otherwise print error massage
        if self.num_stages_edit.text() != '' :
            # self.resize(1100,350)
            if int(self.num_stages_edit.text())> 20 or int(self.num_stages_edit.text())  <1 :
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Critical)
                msg.setText("input invalid")
                msg.setInformativeText('please choose a number between 1 and 20')
                msg.setWindowTitle("Error")
                msg.exec_()
            
            else :
            ### reset all widgets out of window 1 ###
                clear_layout(self.grid_advanced_group_box)
                clear_layout(self.grid1)
                myFont=QFont()
                myFont.setBold(True)
                label = QLabel('current num stages : ')
                label.setFont(myFont)
                self.chBox.setFont(myFont)
                self.advanced_group_box.setFont(myFont)
                self.grid1.addWidget(label, 1, 0)
                self.grid1.addWidget(self.num_stages_edit, 1, 1)
                self.grid1.addWidget(self.chBox, 1, 2)
                self.grid1.addWidget(self.advanced_group_box,2,2,2,5)

                ### setup layout and add probability widgets to the checkbox ###
                
 

                self.advanced_group_box_widgets = {'vmax':[QSlider(Qt.Horizontal),QLabel('')],'chatting probability':[QSlider(Qt.Horizontal),QLabel('')],
                'tell story probability':[QSlider(Qt.Horizontal),QLabel('')],'group talking probability':[QSlider(Qt.Horizontal),QLabel('')]
                ,'talking and walking probability':[QSlider(Qt.Horizontal),QLabel('')], 'requesting service probability':[QSlider(Qt.Horizontal),QLabel('')]
                ,'max talking distance':[QSlider(Qt.Horizontal),QLabel('')],'max servicing radius':[QSlider(Qt.Horizontal),QLabel('')],'talking base time':[QSlider(Qt.Horizontal),QLabel('')],'tell story base time':[QSlider(Qt.Horizontal),QLabel('')]
                ,'group talking base time':[QSlider(Qt.Horizontal),QLabel('')],'talking and walking base time':[QSlider(Qt.Horizontal),QLabel('')],'receiving service base time':[QSlider(Qt.Horizontal),QLabel('')]
                ,'requesting service base time':[QSlider(Qt.Horizontal),QLabel('')],'use danger zone':[QSlider(Qt.Horizontal),QLabel('')]}

                for i,item in enumerate(list(self.advanced_group_box_widgets.items())):
                    self.grid_advanced_group_box.addWidget(QLabel(item[0]),i,0)
                    self.grid_advanced_group_box.addWidget(item[1][0],i,1)
                    item[1][0].setTickPosition(1) 
                    self.grid_advanced_group_box.addWidget(item[1][1],i,2)
                    item[1][0].setMinimum(0)
                    item[1][0].setMaximum(10)
                    item[1][0].setObjectName(item[0])
                    item[1][0].valueChanged.connect(self.updateLabel)
                    ### calculate the values back ###
                    value = 0
                    if item[0] in ['vmax'] :
                        value = ( w.advanced_configs[item[0]] -1)* 10 +0.1
                    elif item[0] in ['max talking distance','max servicing radius'] :
                        value =( w.advanced_configs[item[0]]-1)* 5                    
                    elif item[0] in ['chatting probability','tell story probability','group talking probability','talking and walking probability'] :
                        value =   w.advanced_configs[item[0]]*10            
                    elif item[0] in ['requesting service probability'] :
                        value =   w.advanced_configs[item[0]]*500                  
                    elif item[0] in ['use danger zone'] :
                        value =   w.advanced_configs[item[0]]
                        item[1][0].setMinimum(0)
                        item[1][0].setMaximum(1)
                   
                    else :
                        value =   w.advanced_configs[item[0]] / 2
                    item[1][0].setValue(value)
                   

                self.advanced_group_box.show()
                self.advanced_group_box.hide()

                ### connect functions with checkbox to show hide the advanced box  ###
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
                w.obstacles_spawning_parameters = dict()
                stages_values = list(w._stages.values())
                self.num_obstacles_widgets =[]
                for i in range(int(self.num_stages_edit.text())) :         
                    self.new_widgets = [QPushButton('stage num '+ str(i+1)),QSpinBox(),QSpinBox(),QSpinBox()]
                    self.new_widgets[0].clicked.connect(partial(self.on_button_stage_clicked,'Stage '+ str(i+1),i))
                    self.num_obstacles_widgets.append(self.new_widgets)
                    self.training_curriculum_widgets = self.training_curriculum_widgets +[self.new_widgets]
                    if i < len(stages_values)  and stages_values[i] is not None :
                        self.new_widgets[1].setValue(stages_values[i]['static'])
                        self.new_widgets[1].setObjectName(str(i))
                        self.new_widgets[1].textChanged.connect(self.update_obstacles_spawning_parameters)
                        self.new_widgets[2].setValue(stages_values[i]['dynamic_human'])
                        self.new_widgets[2].setObjectName(str(i))
                        self.new_widgets[2].textChanged.connect(self.update_obstacles_spawning_parameters)
                        self.new_widgets[3].setValue(stages_values[i]['dynamic_robot'])
                        self.new_widgets[3].setObjectName(str(i))
                        self.new_widgets[3].textChanged.connect(self.update_obstacles_spawning_parameters)
                        
                    for j in range(4):
                        grid.addWidget(self.new_widgets[j], i+2, j,Qt.AlignTop)
                   
                    available_models_copy = copy.deepcopy(w.available_models)
                    for x, available_models_copy_key in enumerate(list(available_models_copy.keys())) : 
                        obstacles_count = self.new_widgets[x+1].value()
                       
                        ### check if ther no available models fro mthis type and divide th num obstacles in window on equally upon the duffrent available model of each type ###
                        if available_models_copy[available_models_copy_key] is not  None :
                            
                            for item in  list(available_models_copy[available_models_copy_key].items()) :
                                available_models_copy[available_models_copy_key][ item[0]]= [0,item[1]]
                                
                            while  obstacles_count> 0 :
                                for item in  list(available_models_copy[available_models_copy_key].items()) :
                                    if  obstacles_count== 0 :
                                        break
                                    available_models_copy[available_models_copy_key][ item[0]][0]= item[1][0]+1                        
                                    obstacles_count = obstacles_count -1
                          
                    w.obstacles_spawning_parameters[i+1] = available_models_copy
                    
                    

                        

                self.grid1.addWidget(self.group_box_obstacles,2,0,2,2)

    def update_obstacles_spawning_parameters(self) :
        i = int(self.sender().objectName())
        widgets= self.num_obstacles_widgets[i]
        available_models_copy = copy.deepcopy(w.available_models)
        for x, available_models_copy_key in enumerate(list(available_models_copy.keys())) : 
                    obstacles_count = widgets[x+1].value()
                    
                    ### check if ther no available models fro mthis type and divide th num obstacles in window on equally upon the duffrent available model of each type ###
                    if available_models_copy[available_models_copy_key] is not  None :
                        
                        for item in  list(available_models_copy[available_models_copy_key].items()) :
                            available_models_copy[available_models_copy_key][ item[0]]= [0,item[1]]
                            
                        while  obstacles_count> 0 :
                            for item in  list(available_models_copy[available_models_copy_key].items()) :
                                if  obstacles_count== 0 :
                                    break
                                available_models_copy[available_models_copy_key][ item[0]][0]= item[1][0]+1                        
                                obstacles_count = obstacles_count -1
                        
        w.obstacles_spawning_parameters[i+1] = available_models_copy
    def updateLabel(self,value):

               
        key = self.sender().objectName()

        if key in ['vmax'] :
            value = 1 +  value/ 10
        elif key in ['max talking distance','max servicing radius'] :
            value = 1+ value/ 5
        
        elif key in ['chatting probability','tell story probability','group talking probability','talking and walking probability'] :
            value =   value/10
        elif key in ['requesting service probability'] :
            value =   value/500 
        elif key in ['use danger zone'] :
            value =   value== 1.0
        else :
            value =   value * 2

         
        self.advanced_group_box_widgets[key][1].setText(str(value))    
   

    def on_button_advanced_configs_clicked(self,s):
        if s == Qt.Checked:
            self.advanced_group_box.show()
            self.resize(1100,350)
        else: 
            self.advanced_group_box.hide()
            self.resize(350,350)

    def on_button_stage_clicked(self, button_name,i):

        if not (self is  None):

            ### call a sub window and label it ###
            self.window_extended_stage= window_extended_stage()
            self.window_extended_stage.set_up_ui_window_extended_stage(button_name,int(self.training_curriculum_widgets[i][1].text()),int(self.training_curriculum_widgets[i][2].text()),int(self.training_curriculum_widgets[i][3].text()))
            self.window_extended_stage.show()

        else:
            self.close()  # Close window.
            self = None  # Discard reference

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
            
            advanced_configs ={}
            for item in list(self.advanced_group_box_widgets.items()) :
                if (item[1][1].text() == "True") :
                    advanced_configs[item[0]]=1.0
                elif (item[1][1].text() == "False") :
                    advanced_configs[item[0]]=0.0
                else :
                    advanced_configs[item[0]]=float(item[1][1].text())


            # self.training_curriculum_widgets
            
            
            
            write_stages_to_yaml(new_stages)
            write_obstacles_spawning_parameters_to_yaml(w.obstacles_spawning_parameters)
            write_advanced_configs_to_yaml(advanced_configs)

            

class window_extended_stage(QWidget):
    def __init__(self):
        super().__init__()

    def set_up_ui_window_extended_stage(self,stage_name,num_static,num_human,num_robot): 
        self.stage_name = stage_name
        self.setObjectName("window_extended_"+stage_name)
        self.setWindowTitle(stage_name + ' extended configs')    
        window_extended_stage_grid = QGridLayout()
        self.setLayout(window_extended_stage_grid)
        
        self.nums_obstacles_array = [num_static,num_human,num_robot]
        nums_obstacles_array_counter = [num_static,num_human,num_robot]
        group_box_dict = {QGroupBox("        static obstacles"):[QGridLayout(),0], QGroupBox("        human obstacles"):[QGridLayout(),1],QGroupBox("        robot obstacles"):[QGridLayout(),2]}
        
        ### get available_models out of yaml Datei and write the values into the group boxes ###
        available_models_values = list( w.available_models.values())
        self.available_models_qspin_boxes_array = []
        for box, layout in group_box_dict.items():
            box.setLayout(layout[0])
            
            if available_models_values[layout[1]] is not None:
                
                ### for every group box write 0 as start value in every QspinBox ###
                available_models_qspin_boxes = []
                for i,available_model in enumerate(list( available_models_values[layout[1]].keys())):
                    layout[0].addWidget(QLabel(available_model),i,0,Qt.AlignTop)
                    available_models_qspin_boxes = available_models_qspin_boxes +[QSpinBox()]
                    available_models_qspin_boxes[-1].setValue(0)
                    layout[0].addWidget(available_models_qspin_boxes[-1],i,1,Qt.AlignTop)
                ### add special case when ther are no available_models_values for this type of obtacles  ###


                self.available_models_qspin_boxes_array = self.available_models_qspin_boxes_array +[available_models_qspin_boxes]

                ### divide the number of obstacles equally on the available models ###
                while  nums_obstacles_array_counter[layout[1]]> 0 :
                    for qspinbox in  available_models_qspin_boxes:
                        if  nums_obstacles_array_counter[layout[1]] == 0 :
                            break
                        qspinbox.setValue(qspinbox.value()+1)
                        nums_obstacles_array_counter[layout[1]] = nums_obstacles_array_counter[layout[1]] -1
            else :   
                tmp = QSpinBox()
                tmp.setValue(0)
                self.available_models_qspin_boxes_array = self.available_models_qspin_boxes_array +[[tmp]]          


                    
            

            window_extended_stage_grid.addWidget(box,0,layout[1],Qt.AlignTop)

    def closeEvent(self, event):
   

        reply = QMessageBox.question(self,
                                            'This program',
                                            'Do you want to save the configs of s'+ self.objectName()[17:]+'?',
                                            QMessageBox.Yes | QMessageBox.No,
                                            QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.name_obstacles_array = ['static obstacles','human obstacles','robot obstacles']
            for i in range(len(self.nums_obstacles_array)) :
                # if self.nums_obstacles_array[i] == 0  :
                #     continue
                
                summed_value= sum(list(map( get_value_qspin_box, self.available_models_qspin_boxes_array[i])))
                if summed_value != self.nums_obstacles_array[i]:
                    msg = QMessageBox()
                    msg.setIcon(QMessageBox.Critical)
                    msg.setText("invalid sum of "+ self.name_obstacles_array[i])
                    msg.setInformativeText('please make the sum of '+ str(self.name_obstacles_array[i]+' equals to '+str(self.nums_obstacles_array[i])))
                    msg.setWindowTitle("Error")
                    msg.exec_()
                    event.ignore()
                    return
            ### write the values to the array to save them to yaml later ###
            for i in range(len(self.nums_obstacles_array)) :
                if self.nums_obstacles_array[i] == 0  :
                    continue
                
                for j,qspin_box in enumerate(self.available_models_qspin_boxes_array[i]) :
                    available_model_name = list(w.obstacles_spawning_parameters[int(self.stage_name[6:])][self.name_obstacles_array[i]].keys())[j]
                    w.obstacles_spawning_parameters[int(self.stage_name[6:])][self.name_obstacles_array[i]][available_model_name][0]= qspin_box.value()

            write_obstacles_spawning_parameters_to_yaml(w.obstacles_spawning_parameters)
               
                            


class main_window(QMainWindow):
    

    def __init__(self):
        super().__init__()

        self.w = None  
        self._stages = dict()
        self.available_models = dict()
        self.advanced_configs = dict()
        self.obstacles_spawning_parameters = dict()

    def set_up_main_window_ui(self): 
        self.window = QWidget(self)
        ### add label, size and center the main window ###
        self.setCentralWidget(self.window)
        self.setWindowTitle('Arena Configs')
        self.resize(450, 200)
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
        self.button2 = QPushButton('Edit Flatland Models')
        self.button2.clicked.connect(self.on_button2_clicked)
        self.button3 = QPushButton('Create New Model')
        self.button3.clicked.connect(self.on_button3_clicked)
        self.button4 = QPushButton('Create Custom Scenario')
        self.button4.clicked.connect(self.on_button4_clicked)
       
        ### add the layout of the main window ###
        layout = QVBoxLayout()
        layout.addWidget(self.button1)
        layout.addWidget(self.button2)
        layout.addWidget(self.button3)
        layout.addWidget(self.button4)
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
            self.flatland_model_editor_window = FlatlandModelEditor()
            self.flatland_model_editor_window.show()
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
    def on_button4_clicked(self, checked):
        if self.w is None:
            self.window4= SubWindow()
            self.window4.setWindowTitle('New scenario Configs')

            self.window4.show()
        else:
            self.w.close()  # Close window.
            self.w = None  # Discard reference.
     

        

app = QApplication([])
app.setStyle('Fusion')
w = main_window()
w.set_up_main_window_ui()

app.exec()



