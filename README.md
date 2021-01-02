# nav_in_flatland

# What is this repository for?
Train DRL agents on ROS compatible simulations for autonomous navigation in highly dynamic environments. Flatland-DRL integration is based on Ronja Gueldenring's repo: drl_local_planner_ros_stable_baselines. Following features are included:

* Setup to train a local planner with reinforcement learning approaches from [stable baselines](https://github.com/hill-a/stable-baselines)

* Training in a simulator fusion of [Flatland](https://github.com/avidbots/flatland) and [pedsim_ros](https://github.com/srl-freiburg/pedsim_ros)

* Local planner has been trained on static and dynamic obstacles: [video](https://www.youtube.com/watch?v=nHvpO0hVnAg)

### Documentation ###
* How to use flatland: http://flatland-simulator.readthedocs.io
* 


## Installation
0. Standard ROS setup (Code has been tested with ROS-melodic on Ubuntu 18.04) with catkin_ws
```
sudo apt-get update && sudo apt-get install -y \
libqt4-dev \
libopencv-dev \
liblua5.2-dev \
screen \
ros-melodic-tf2-geometry-msgs \
ros-melodic-navigation \
ros-melodic-rviz 
```

1. Create a catkin workspace & clone this repo into it
````
mkdir -p catkin_ws/src
cd catkin_ws && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
cd src
git clone https://github.com/ignc-research/arena-rosnav
cd arena-rosnav
````

2. Copy Install file and install forks
````
rosws update
```` 

3. Install virtual environment and wrapper (as root or admin!) on your local pc (without conda activated) to be able to use python3 with ros
```
sudo pip3 install --upgrade pip
sudo pip3 install virtualenv
sudo pip3 install virtualenvwrapper
which virtualenv   # should output /usr/local/bin/virtualenv  
```

      
4. Create venv folder inside hom directory
```
cd $HOME
mkdir python_env   # create a venv folder in your home directory 
```

Add this into your .bashrc/.zshrc :
```
echo "export WORKON_HOME=/home/linh/python_env   #path to your venv folder
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3   #path to your python3 
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
source /usr/local/bin/virtualenvwrapper.sh
source ~/.zsh" >> ~/.zshrc
```
Create a new venv
```
mkvirtualenv --python=python3.6 rosnav
workon arena-flatland-py3
```


### install forks
````
cd nav_in_flatland
rosws update
````

### catkin_make
````
go to catkin_ws
catkin_make
````

### quick start simulation env and launch
````
roslaunch flatland_bringup start_flatland.launch  train_mode:=false
````


###  start plan manager with FSM
````
rosrun plan_manage plan_manager_node
````

### [optional]use task generator to spawn random obstacles
````
rosrun task_generator task_generator.py 
````

### [optional]use flatland_gym_env
````
roslaunch flatland_bringup start_flatland.launch  train_mode:=true

rosrun plan_local_drl flatland_gym_env.py

````


### Structure of the packges
1. flatland bringup: final launch file
2. nav: 
   1. plan_global
   2. plan_local
   3. plan_manage
   4. plan_manage_move_base
   5. plan_msgs
3. simulator_setup
   1. maps
   2. obstacles
   3. robot
4. task_generator
5. utils
   1. rviz_plugin
