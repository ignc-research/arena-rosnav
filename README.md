# arena-rosnav

<p align="center">
  <img width="300" height="300" src="/img/dynamic1.gif">
	<img width="300" height="300" src="/img/1training.gif">
</p>


# What is this repository for?
Train DRL agents on ROS compatible simulations for autonomous navigation in highly dynamic environments. Test state of the art local and global planners in ROS environments both in simulation and on real hardware. Following features are included:

* Setup to train a local planner with reinforcement learning approaches from [stable baselines](https://github.com/hill-a/stable-baselines)

* Training in simulator [Flatland](https://github.com/avidbots/flatland) in train mode

* Local planner has been trained on static and dynamic obstacles with highly dynamic tasks

* Implementation of intermediate planner classes to combine local DRL with global map-based planning of ROS Navigation stack

* Integration of other obstacle avoidance approaches in ROS 

* Testing a variety of planners (learning based and classic) within specific scenarios in test mode

* Modular structure for extension of new functionalities and approaches

### Documentation ###
* How to use flatland: http://flatland-simulator.readthedocs.io
* Full documentation and system design is released this week

## Installation
0. Standard ROS setup (Code has been tested with ROS-melodic on Ubuntu 18.04) with catkin_ws
Install ROS Melodic following the steps from ros wiki:
```
http://wiki.ros.org/melodic/Installation/Ubuntu
```
Install additional ros pkgs 
```
sudo apt-get update && sudo apt-get install -y \
libqt4-dev \
libopencv-dev \
liblua5.2-dev \
screen \
python3-catkin-pkg-modules \
python3-rospkg-modules \
python3-empy
```

1. Create a catkin_ws and clone this repo into your catkin_ws 
````
cd $HOME
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone https://github.com/ignc-research/arena-rosnav
cd arena-rosnav && rosws update
./geometry2_install.sh
source $HOME/.zshrc
cd ../.. 
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.zsh
````
2. To be able to use python3 with ROS, you need an virtual environment. We recommend using venv. Install virtual environment and wrapper (as root or admin! with sudo) on your local pc (without conda activated, deactivate conda env. if you have one active)
```
sudo pip3 install --upgrade pip
sudo pip3 install virtualenv
sudo pip3 install virtualenvwrapper
which virtualenv   # should output /usr/local/bin/virtualenv  
```
      
3. Create venv folder inside your home directory
```
cd $HOME
mkdir python_env   # create a venv folder in your home directory 
```

4. Add exports into your .zshrc (if you use bash change the last line to bashrc instead of zshrc):
```
echo "export WORKON_HOME=/home/linh/python_env   #path to your venv folder
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3   #path to your python3 
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
source /usr/local/bin/virtualenvwrapper.sh" >> ~/.zshrc
```
5. Create a new venv
```
mkvirtualenv --python=python3.6 rosnav
workon rosnav
```

6. Install packages inside your venv (venv always activated!):
```
pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf tf2_ros --ignore-installed
pip install pyyaml catkin_pkg netifaces
```     

Install stable_baselines3 for training DRL into your venv (venv always activated!)
```
cd $HOME/catkin_ws/src/forks/stable-baselines3
pip install -e .
```

## Usage
Before you test out the packages, always source your setup.zsh /setup.bash inside your catkin workspace also source your $HOME/.zshrc:
```
cd $HOME/catkin_ws
source devel/setup.zsh
source $HOME/.zshrc
```
### quick start simulation env and launch
````
roslaunch flatland_bringup start_flatland.launch  train_mode:=false
````
### quick test with the training 
In one terminnal
```bash
roslaunch flatland_bringup start_flatland.launch  train_mode:=true
```
In another terminal
```
roscd flatland_local_planner_drl 
python scripts/training/training_example.py
```
Hint: During 2021-01-05 and 2021-01-10, plan_local_drl package is still under the development, which means the api of the class could be drastically changed. Sorry about the inconvinience!

###  start plan manager with FSM
````
rosrun plan_manage plan_manager_node
````

### use task generator to spawn random obstacles
````
roslaunch flatland_bringup start_flatland.launch  train_mode:=false
rosrun task_generator task_generator_node.py 
````
Now you can manually generate new tasks using the Pluggin inside RVIZ "Generate Task". You should set a "Flatland Goal" with the button and afterwards a "2D navigation goal". Afterwards the robot will automatically move once you spawn a new tasks by clicking the "Generate Task" button.

### use flatland_gym_env
````
roslaunch flatland_bringup start_flatland.launch  train_mode:=true

rosrun plan_local_drl flatland_gym_env.py

````


## Structure of the project
1. Bringup: 
   1. Configs (configs with parameters
   2. Launchfiles
   3. RVIZfiles
2. nav: 
   1. plan_global (move base)
   2. plan_local
      1. training-based
         1. DRL
            1. scripts
               1. training
               2. testing
            2. rl_agent
               1. envs
               2. utils (reward collector, state collector, action collector , ..)
            3. models (neural networks and policies stable baselines)    
         2. Immitation Learning
            1. scripts
               1. dataset_gen
               2. training
               3. testing
            2. weights
         3. weights
      2. other-planners 
         1. cadrl
         2. mcl
         3. ...
   3. plan_manager
      1. waypoint-generators
   4. plan_localization
   5. plan_manager_move_base
   6. plan_msgs
3. simulator
   1. maps
   2. models
      1. robot
      2. obstacles
   3. scripts
      1. behavior_modeling
      2. heatmap_gen
4. task_generator
   1. scripts
5. utils
   1. rviz_plugin
   2. pluggins
6. evaluation
   1. scripts
   
   
   
   
### 5. Navigation framework

<p align="center">
  <img width="500" height="300" src="img/plan_manager.png">
</p>

#### **arena_navigation**
   1. **fake_localization**(pkg) 
   2. **mapping**:
      1. costmap2D(pkg) 
      2. Euclean Signed Distancefield Map(pkg) 
      3. Topology Graph(pkg) 
      4. Voxgraph(pkg) 
      5. ...
   3. **global_planner**
      1. arena_global_planner_Dijkstra(pkg) 
      2. arena_global_planner_Astar(pkg) 
      3. arena_global_planner_JPS(Jump point search)(pkg) 
      4. arena_global_planner_KinoAstar(pkg)  
      5. arena_global_planner_Informed_RRTstar(pkg) 
      6. ...
   4. **local_planner**
      1. learning_based
         1. arena_local_planner_drl(pkg) 
         2. arena_local_planner_cardl(pkg)
         3. ... 
      2. model_based
         1. arena_local_planner_TEB(pkg) 
         2. arena_local_planner_VFH*(pkg) 
         3. ...
   5. **plan_manager**(pkg) 
      1. plan_collector
      2. plan_manager
      3. plan_manager_node
   6. **plan_msgs**(pkg) 
      1. msg
         1. RobotState.msg
      2. srv
         1. Subgoal.srv

Plan manager
* plan_manager_node will init a ros node for plan_manager
* plan_manager is implemented as a Finite State Machine
* plan_manager is responsible for state transfer, ros communication and call plan functions from plan_collecor

Plan collector
* plan_collector has no ros communication tasks, plan_collecor only responsible for algorithms
* plan_collector calls libraries from other pkgs(e.g. pkgs in mapping, local planner, global planner) to achieve its functions
* plan_collector also responsible for subgoal generation, which is the job of intermediate planner.

Plan msgs
* saves user-defined msg or srv for arena navigation


### 6. Simulator: Flatland
[Flatland](https://github.com/avidbots/flatland) is a 2D physical simulator based on box2D, which is made to be integratable with ROS and easy to extend functions with its plugin mechanism.

In our project, we have modified and extended the original Flatland source repositary in order to make it better suitable to our DRL planning purpose. The parts that have been modified will be cleared somehow in following sections.

A great introduction to flatland is listed in following website, please checi it out (most importantly in order to know how to create plugin in flatland):
* How to use flatland: http://flatland-simulator.readthedocs.io

Things need to know:
* How flatland updates its simulation progress
* How to write model .yaml files for flatland
* How to create flatland plugins(e.g. laser, driver, motion behavior) which can be added to the model .yaml file


##### How flatland updates its simulation progress
````
flatland_server/src/flatland_server_node.cpp
flatland_server/src/simulation_manager.cpp         (modified by our project)
flatland_server/src/world.cpp
flatland_server/src/timekeeper.cpp
flatland_plugins/src/laser.cpp                     (modified by our project)
````
check out these files, everything relative to simulation update is contained there.
We made some modification in *simulation_manager.cpp*, where we create a */step_world* service server.

##### How to write model .yaml files for flatland
Robot, Obstacles and world can be described by .yaml files, which provide easy setting to users.

check out the model section in http://flatland-simulator.readthedocs.io

##### How to create flatland plugins
Sensors such as laser, actuator such ad diff_driver & other user defined motion behaviors can be coded as a flatland plugin and added to the model .yaml file.

check out the plugin section in http://flatland-simulator.readthedocs.io

````
flatland_plugins/src/laser.cpp                     (modified by our project)
flatland_plugins/src/diff_drive.cpp                (modified by our project)
flatland_plugins/src/model_tf_publisher.cpp        (modified by our project)
flatland_plugins/include/flatland_plugins/tween.h  (for dynamic obstacle motion behavior)
flatland_plugins/include/flatland_plugins/update_timer.h
````
These are the plugins that currently we are using and some of them are modified.

Modification are mostly done in these two functions in each plugins.
These change are made intended to make the publication of topics done in *AfterPhysicsStep* otherthan in *BeforePhysicsStep*.

````
void BeforePhysicsStep(const Timekeeper& timekeeper);
void AfterPhysicsStep(const Timekeeper &timekeeper) ;
````

### 7. Task Generator
To be added...

### 8. DRL Local planner(Training and Testing)
<p align="center">
  <img width="400" height="300" src="img/local_planner.png">
  <img width="400" height="300" src="img/synchronization.png">
</p>

##### Communication:
DRL local planner get the needed observation info by using ROS communication. This may slows down the training, but for current version we just keep it.

DRL local planner get observation info from:
   * flatland server: laser scan
   * plan manager: robot state, subgoal

DRL local planner send action command to flatland server
   * flatland server: diff_drive

##### Observation synchronization
DRL local planner contains observation collector and we designed a synchronization mechanism for following important reasons & aspects:
   1. In real world, each sensor has its own publishing rate and are different from each other
   2. The action calculation should based on the observations that are synchronized, otherwise is useless.
   3. The calculated action is only valid for a specified time horizon(control horizon),e.g. 0.2s. For different control horizon, the action space should be different. 
      1. example 1: action is calculated every 0.01s, time horizon=0.01s, suppose calculated action=1m/s, in this time horizon the robot will actually move 0.01m.
      2. example 2: action is calculated every 0.5s, time horizon=0.5s, suppose calculated action=1m/s, in this time horizon the robot will actually move 0.5m.
      * From 1 & 2, one can see for a same action space, a different time horizon will result in different actual result.


To be added...

### 9. Utils
contains rviz_plugins & planning visulizations needed to be showed in rviz.

