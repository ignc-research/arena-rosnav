# General Information
arena configs gui is a tool designed to automaise and simplify the process of adapting the agent training process to the customer needs 


 <pre>

   <img width="400" height="250" src="/img/main_gui.jpg"> 

                  Gui main window   
</pre>

Main features are : 
- Editing the training curriculum 
- Creating a new agent model
- Creating a custom scenario for evaluation


## Getting started
Navigate to the gui directory and start the gui 
Disclaimer! : please use python3 and preinstall pyqt5 to avoid any conflicts 
```
cd ~/catkin_ws/src/arena-rosnav/arena_configs_gui.py
python ./arena_configs_gui.py

```



## Usage

- Edit curriculum 

| <img width="400" height="250" src="/img/edit_curriculum.jpg"> | <img width="400" height="250" src="/img/advanced_stage.jpg"> | 
|:--:|:--:|
| *Edit curriculum window* | *advanced_stage configs window*|


Here the user can adjust the following :
- Number of stages between 1 to 20 
- Number of obstacles in each satge (static,human and robots) 
- Propabilities of certain actions in the advanced window , max velocity, base time of each action and max talking distance (radius of agetns wher the start to interact with eachother)
- 



