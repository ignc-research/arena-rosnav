from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPalette , QIcon
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
 
    def __init__(self):
        super().__init__()

        self.w = None  
        self.window = QWidget(self)
        ### add label, size and center the main window ###
        self.setCentralWidget(self.window)
        self.setWindowTitle('Arena Configs')
        self.resize(350, 150)
        qtRectangle = self.window.frameGeometry()
        centerPoint = QDesktopWidget().availableGeometry().center()
        qtRectangle.moveCenter(centerPoint)
        self.move(qtRectangle.topLeft())
        layout = QVBoxLayout()

        ### add diffrent buttons to the main window and link them to functions ###
        self.button1 = QPushButton('Edit Training Curriculum')
        self.button1.clicked.connect(self.on_button1_clicked)
        self.button2 = QPushButton('Edit Obstacles Profiles')
        self.button2.clicked.connect(self.on_button2_clicked)
        self.button3 = QPushButton('Create New Model')
        self.button3.clicked.connect(self.on_button3_clicked)
       
        ### add the layout of the first window ###
        layout.addWidget(self.button1)
        layout.addWidget(self.button2)
        layout.addWidget(self.button3)
        self.window.setLayout(layout)

   
    def on_button1_clicked(self, checked):

        ### keep the main window alive with the if statment ###
        if self.w is None:

            ### get current curriculum setting ###
            self._stages = dict()
            self._read_stages_from_yaml()
            print(self._stages)

            ### call a sub window and label it ###
            self.window1 = SubWindow()
            self.window1.resize(250,350)
            self.window1.setWindowTitle('Training Curriculum Configs')

            title = QLabel('titel')
            author = QLabel('Author')
            review = QLabel('Review')

            titleEdit = QLineEdit()
            authorEdit = QLineEdit()
            reviewEdit = QTextEdit()

            titleEdit1 = QLineEdit()

            grid = QGridLayout()
            grid.setSpacing(10)

            grid.addWidget(title, 1, 0)
            grid.addWidget(titleEdit, 1, 1)
            grid.addWidget(titleEdit1, 1, 2)

            grid.addWidget(author, 2, 0)
            grid.addWidget(authorEdit, 2, 1)

            grid.addWidget(review, 3, 0)
            grid.addWidget(reviewEdit, 3, 1, 5, 1)

            self.window1.setLayout(grid)
            self.window1.show()
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

    

        

app = QApplication([])
app.setStyle('Fusion')
w = MainWindow()
w.show()

app.exec()



