# Arena Evaluation
The Arena Evaluation package consists of 3 parts:
- data recording
- data transformation and evaluation
- plotting

## 01 Data Recording
To record data as csv file while doing evaluation runs set the flag `use_recorder:="true"` in your `roslaunch` command. For example:

```
roslaunch arena_bringup start_arena_flatland.launch disable_scenario:="false" map_file:="map1" scenario_file:="eval/obstacle_map1_obs05.json" local_planner:="teb" use_recorder:="true"
```

The data will be recorded in `.../catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/evaluation/arena_evaluation/01_recording`.
The script stops recording as soon as the agent finishes the scenario and stops moving. After doing all evaluation runs please remember to push your csv files.

Note: Sometimes csv files will be ignored by git so you have to use git add -f <file>. We recommend using the code below.
```
roscd arena_evaluation
git add -f .
git commit -m "evaluation run"
git pull
git push
```

## 02 Data Transformation and Evaluation
TODO
