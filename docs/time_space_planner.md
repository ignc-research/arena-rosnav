
## Try in empty map
#### 1) Start Simulation
```
roslaunch arena_bringup start_arena_flatland.launch
```
#### 2) Start time_space plan manager(fsm:Finite state machine) 
```
roslaunch arena_bringup timed_space_planner_fsm.launch
```

## Test in automatic test mode 
#### 1) Start Simulation
```
roslaunch arena_bringup start_arena_flatland.launch map_file:="map1" scenario_file:="eval/obstacle_map1_obs20.json" local_planner:="mpc" disable_scenario:="false"
```
#### 2) just wait 
(wait all the obstacles are loaded by task generator, in order to wait all topics in ros master are ready)
#### 3) Start time_space plan manager 
```
roslaunch arena_bringup timed_space_planner_fsm.launch
```

## Parameter.yaml for plan manager(fsm:Finite state machine) 
The paramater yaml file is located at:
```
src/arena-rosnav/arena_bringup/launch/plan_fsm_param.yaml
```

the only parameter that are relavant to DRL:
* use_drl:
  *  true:  cmd_vel is published by drl or by mpc planners
  *  false: cmd_vel is published by traj tracker in plan_fsm
* subgoal_mode:
  * 0:  spacial_horizon
  * 1:  timed_space
  * 2:  simple_sample

