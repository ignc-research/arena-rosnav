# Implementation of new Robot Models

1. **Add a new robot flatland model**:

- Add it to `~/catkin_ws/src/arena-rosnav/simulator_setup/robot` (navigate there to see examples)
- Refer to the [official flatland documentation](https://flatland-simulator.readthedocs.io/en/latest/flatland_tutorials/create_model.html) for custom model implementation
- Laser specifications are parsed to the ObservationCollector automatically (read from the robot model yaml)

2. **Add robot name to launch files**:

- Add specifications to `start_arena_flatland.launch` and `start_training.launch`

```roslaunch
<arg name='radius' default='0.347' if="$(eval arg('model') == 'youbot')"/>
<arg name='speed' default='0.8' if="$(eval arg('model') == 'youbot')"/>
```

- The model parameter for the new robot has to be the same as its folder name

3. **Add action space specifications**:

- The file has to be in .yaml and must follow the following layout:

```yaml
robot:
  holonomic: True
  discrete_actions:
    - name: move_forward
      linear: 2.78
      angular: 0.0
    - name: move_backward
      linear: -0.15
      angular: 0.0
    - name: turn_left
      linear: 0.15
      angular: 0.35
    - name: turn_right
      linear: 0.15
      angular: -0.35
    - name: turn_strong_left
      linear: 0.0
      angular: 0.75
    - name: turn_strong_right
      linear: 0.0
      angular: -0.75
    - name: stop
      linear: 0.0
      angular: 0.0
  continuous_actions:
    linear_range:
      x: [-2.78, 2.78] # Maximum translational velocity in m/s
      y: [-2.78, 2.78]
    angular_range: [-1.0, 1.0] # Maximum rotational velocity in rad/s
```

- The file needs to be named after the robot model folder and added to `~/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/configs`

4. **Add conventional planner parameter files in order to enable evaluation with conventional navigation approaches**

- Add them to [~/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/conventional/config ](https://github.com/ignc-research/arena-rosnav/tree/local_planner_subgoalmode/arena_navigation/arena_local_planner/model_based/conventional/config)
- (see examples in the directory above)
- The subdirectory has to be named after the robot model and must contain of the following 6 files:
  - base_local_planner_params.yaml
  - costmap_common_params.yaml
  - global_costmap_params.yaml
  - local_costmap_params.yaml
  - move_base_params.yaml
  - plan_fsm_param.yaml
- Usually, one can copy most of the parameters from existing folders. Make sure to adjust robot model relevant parameters though!

5. **Start the simulation with the new robot model**:

- by parsing its name to the "model:=" parameter of the launch file

#### Note:

So far, the observation space only incorporates laser information, distance and relative angle to goal and optionally the most recent velocities.
