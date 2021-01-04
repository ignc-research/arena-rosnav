# nav_in_flatland

# What is this repository for?
Train DRL agents on ROS compatible simulations for autonomous navigation in highly dynamic environments. Flatland-DRL integration is based on Ronja Gueldenring's repo: drl_local_planner_ros_stable_baselines. Following features are included:

* Setup to train a local planner with reinforcement learning approaches from [stable baselines](https://github.com/hill-a/stable-baselines)

* Training in a simulator fusion of [Flatland](https://github.com/avidbots/flatland) and [pedsim_ros](https://github.com/srl-freiburg/pedsim_ros)

* Local planner has been trained on static and dynamic obstacles: [video](https://www.youtube.com/watch?v=nHvpO0hVnAg)

### Documentation ###
* How to use flatland: http://flatland-simulator.readthedocs.io
* 
### create workspace & clone

````
mkdir -P catkin_ws/src
cd catkin_ws/src
git clone https://github.com/FranklinBF/navigation_flatland.git
````

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
### geometry2
The official ros only support python2. In order to make the $tf$ work in python3, its necessary to compile it with python3. We provided a script to automately this this
and do some additional configurations for the convenience . You can simply run it with 
````bash
./geometry2_install.sh
After that you can try to import tf in python3 and no error is supposed to be shown up.
````


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
roscd plan_local_drl
python scripts/training/training_example.py
```
Hint: During 2021-01-05 and 2021-01-10, plan_local_drl package is still under the development, which means the api of the class could be drastically changed. Sorry about the inconvinience!

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
