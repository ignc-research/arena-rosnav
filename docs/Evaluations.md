## Evaluations
We provide tools to evaluate the planners.
### Usage
#### 1) Start Simulation
```
roslaunch arena_bringup start_arena_flatland.launch disable_scenario:="false" map_file:="map1" scenario_file:="eval/obstacle_map1_obs10.json" local_planner:="teb"
```
Explanation:
* disable_scenario:="false": to start a scenario (otherwise only the map will be loaded without additional obstacles / goals)
* map_file:="map1": select a map file (ie map_file:="map_empty") for the map without static obstacles
* scenario_file:="eval/obstacle_map1_obs10.json": select the corresponding scenario file
* local_planner:="teb": choose between "dwa", "teb", "mpc", "cadrl"
* in order to change the velocity of the dynamic obstacles, the scenario file must be edited
#### 2) Record Rosbags
```
rosbag record -o cadrl_map1_ob10_vel_01 /scenario_reset -e "(.*)police(.*)"
```
Explanation:
* this command will record all topics necessary for evaluation
* -e "(.*)police(.*)": records all topics containing "police"
* cadrl_map1_ob10_vel_01: name of bag file, in this example the cadrl planner was recorded in map1 with 10 obstacles with lin_vel = 0.1

#### After one test run is over, stop the rosbag command and go to the scenario.json files. Change the linear velocity of every dynamic obstacle in the scenario.json file and start from step 1) again. (start simulation and start rosbag command again). If there are any issues while recording and you want to record a new recording, just stop the rosbag command and simulation and start over again. The rosbags will be saved as a new file. 
