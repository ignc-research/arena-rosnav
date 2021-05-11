# importing libraries
from PyQt5.QtWidgets import * 
from PyQt5 import QtCore, QtGui
from PyQt5.QtGui import * 
from PyQt5.QtCore import * 
import sys
  
  
class Window(QMainWindow):
  
    def __init__(self):
        super().__init__()
  
        # setting title
        self.setWindowTitle("Python ")
  
        # setting geometry
        self.setGeometry(100, 100, 500, 400)
  
        # calling method
        self.UiComponents()
  
        # showing all the widgets
        self.show()
  
    # method for components
    def UiComponents(self):
  
        # creating QDial object
        dial = QDial(self)
  
        # setting geometry to the dial
        dial.setGeometry(100, 100, 100, 100)
  
        # making notch visible
        dial.setNotchesVisible(True)
  
        # value
        value = 50
  
        # setting value to the dial slider
        dial.setValue(value)
  
  
        # creating a label
        label = QLabel("GeeksforGeeks", self)
  
        # setting geometry to the label
        label.setGeometry(220, 125, 200, 60)
  
        # making label multiline
        label.setWordWrap(True)
  
        # getting slider value
        value = dial.value()
  
        # setting text to the label
        label.setText("Value : " + str(value))
  
        # getting value changed signal
        dial.valueChanged.connect(lambda: label.setText("Value = " + str(dial.value())))
  
  
  
# create pyqt5 app
App = QApplication(sys.argv)
  
# create the instance of our Window
window = Window()
  
# start the app
sys.exit(App.exec())