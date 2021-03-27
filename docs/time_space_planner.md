## Preparations
```
git checkout local_planner_subgoalmode
git pull

cd $HOME/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## Try in empty map
#### 1) Start Simulation in one terminal
```
workon rosnav
roslaunch arena_bringup start_arena_flatland.launch
```
#### 2) Start time_space plan manager(fsm:Finite state machine) in another terminal
```
workon rosnav
roslaunch arena_bringup timed_space_planner_fsm.launch
```
Note: for this simple test use_drl must be set to true (look below in section: Setting parameter for plan manager)

## Test in automatic test mode 
#### 1) Start Simulation in one terminal
```
workon rosnav
roslaunch arena_bringup start_arena_flatland.launch map_file:="map1" scenario_file:="eval/obstacle_map1_obs20.json" local_planner:="mpc" disable_scenario:="false"
```
#### 2) just wait 
(wait until all the obstacles are loaded by task generator, meanwhile all topics in ros master are ready)
#### 3) Start time_space plan manager in another terminal
```
workon rosnav
roslaunch arena_bringup timed_space_planner_fsm.launch
```

## Setting parameter for plan manager(fsm:Finite state machine) 
Open the parameter yaml file with the following code in VSC:
```
code -r $HOME/catkin_ws/src/arena-rosnav/arena_bringup/launch/plan_fsm_param.yaml
```

The only parameter that are relevant to DRL: (line 1-3)
* use_drl:
  *  true:  cmd_vel is published by drl or by mpc planners
  *  false: cmd_vel is published by traj tracker in plan_fsm
* subgoal_mode:
  * 0:  spacial_horizon
  * 1:  timed_space
  * 2:  simple_sample


## Setting parameter for start flatland launch file simulator
Open the start_arena_flatland file with the following code in VSC:
```
code -r $HOME/catkin_ws/src/arena-rosnav/arena_bringup/launch/start_arena_flatland.launch
```

The important parameter that need to be set: (line 43-44)

* "update_rate"     default="1000.0"
* "step_size"       default="0.005"

```
Remark 1: step_size should smaller than 0.01, e.g. 0.005
Remark 2: update_rate should be compatible with step_size and your computer performance

```
