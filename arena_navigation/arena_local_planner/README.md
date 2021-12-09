# Arena-Rosnav

A flexible, high-performance 2D simulator with configurable agents, multiple sensors, and benchmark scenarios for testing robotic navigation. 

Arena-Rosnav uses Flatland as the core simulator and is a modular high-level library for end-to-end experiments in embodied AI -- defining embodied AI tasks (e.g. navigation, obstacle avoidance, behavior cloning), training agents (via imitation or reinforcement learning, or no learning at all using conventional approaches like DWA, TEB or MPC), and benchmarking their performance on the defined tasks using standard metrics.


| <img width="400" height="400" src="/img/rosnav1.gif"> | <img width="400" height="400" src="/img/rosnav2.gif"> |
|:--:| :--:| 
| *Training Stage* | *Deployment Stage* |


## Local-Planner

In ROS, robot navigation is realized by the global and local planner. The idea of this repository is to implement various local planners and evaluate them in a consistent way.  We want to use already existing local planners for ROS such as dwa, teb, mpc etc and also implement some obstacle avoidance approaches which are not inherently available in ROS.

## 2. Usage

````
How to start:
   start flatland with default map and planner
   1. roslaunch arena_bringup start_arena_flatland.launch       
   start flatland chosing a local planner and a map 
   2. roslaunch arena_bringup start_arena_flatland.launch map_file:="map_empty" local_planner:="teb"     

It should be noted that certain planners may need an individual environment to run. The environmets tested with these planners can be found in the "env" folder. 

### 2.3. [Quick start] test DRL training
Export turtlebot model for simulation 

* In one terminnal, export turtlebot model and start simulation

```bash

roslaunch arena_bringup start_arena_flatland.launch  train_mode:=true 	use_viz:=true  task_mode=random

```
* In another terminal

```
workon rosnav
roscd arena_local_planner_drl
python scripts/training/training_example.py
```
first **activate your python3 env**, which contains libaraies stable_baseline3, geometry2
then python run the script.

Hint: During 2021-01-05 and 2021-01-10, arena_local_planner_drl package is still under the development, which means the api of the class could be drastically changed. Sorry about the inconvinience!

### 2.4. Rviz plugins:
   <p align="center">
      <img width="600" height="480" src="img/rviz_plugin_intro.png">
   </p>

   1. 2D Nav Goal: triggers move_base action
   2. Spawn Model: load a new model.yaml to flatland simulator
   3. Arena Nav Goal: set (global) goal for arena navigation
   4. Generate Task: change task, which changes the position of obstacles and set a new goal for arena navigation

   
#### 3.2. Navigation framework

<p align="center">
  <img width="500" height="300" src="img/plan_manager.png">
</p>


Plan manager
* plan_manager_node will init a ros node for plan_manager
* plan_manager is implemented as a Finite State Machine
* plan_manager is responsible for state transfer, ros communication and call plan functions from plan_collecor

#### 3.4. Task Generator
To be added...

#### 3.5. Utils
contains rviz_plugins & planning visulizations needed to be showed in rviz.


### 4. DRL Local planner(Training and Testing)
<p align="center">
  <img width="400" height="300" src="img/local_planner.png">
  <img width="400" height="300" src="img/synchronization.png">
</p>
