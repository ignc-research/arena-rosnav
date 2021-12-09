## Evaluations
We provide tools to evaluate the planners.
## Usage
### Recording
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

#### 3) Verify the simulation

Once the simulation is started, you need to check if these three topics are published:
``` 
rostopic list
```
* /scenario_reset
* /sensorsim/police/collision
* /sensorsim/police/odom

When using "teb", "dwa" or "mpc" you need to start the scenario by manually putting a "2D Nav Goal" once. After each reset the goal will automatically set. If everything worked you can continue to step 2)

#### 2) Record Rosbags
```
rosbag record -o cadrl_map1_ob10_vel_01 /scenario_reset -e "(.*)police(.*)"
```
Explanation:
* this command will record all topics necessary for evaluation
* -e "(.*)police(.*)": records all topics containing "police"
* cadrl_map1_ob10_vel_01: name of bag file, in this example the cadrl planner was recorded in map1 with 10 obstacles with lin_vel = 0.1

#### After one test run is over, stop the rosbag command and go to the scenario.json files. Change the linear velocity of every dynamic obstacle in the scenario.json file and start from step 1) again. (start simulation and start rosbag command again). If there are any issues while recording and you want to record a new recording, just stop the rosbag command and simulation and start over again. The rosbags will be saved as a new file. 

#### If you want to check the current scenario progress:
```
rostopic echo /scenario_reset
```
This will display the reset count. Once the resets reach the max nr of repeats (set up in the json file), the robot should stay at the goal position. Then you can stop the recording.
## Plotting
Once you have recorded all files, we provide scripts to plot qualitative and quantitative results. Please refer to [Qualitative.md](https://github.com/ignc-research/arena-rosnav/tree/local_planner_subgoalmode/arena_navigation/arena_local_planner/evaluation/readme.md) for qualitative plots (trajectories) and [Quantitative.md](https://github.com/ignc-research/arena-rosnav/tree/local_planner_subgoalmode/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/readme.md) for quantitative plot instructions.

