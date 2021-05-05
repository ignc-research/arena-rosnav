## Demo

#### 1. Normal Arena-Rosnav Installation
See [Installation Manual](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/Installation.md)

#### 2. Switch to sim_to_real branch and pull forks
```
cd ~/catkin_ws/src/arena-rosnav
git checkout sim_to_real
rosws update --delete-changed-uris .
```

#### 3. Start Demo
```
roslaunch arena_bringup pedsim_demo.launch
```
This will start Flatland and Pedestrian Simulator and a script that will spawn a few agents.
