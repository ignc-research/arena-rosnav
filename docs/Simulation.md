## Simulation

# Robots
Arena rosnav incluedes four different robots. 
+ [*turtlebot3-burger*](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)
+ [*jackal*](https://www.clearpathrobotics.com/assets/guides/melodic/jackal/simulation.html)
+ [*ridgeback*](https://www.clearpathrobotics.com/assets/guides/melodic/ridgeback/simulation.html)
+ [*agvota*](https://github.com/inomuh/agv)


All robots are equipt with a laser scanner. The robots differ in size, laser-range etc. See below tabel for more detailed information on each robot:


| Name  | Max Speed (v_x) [_m/s_]  | Max Speed (v_y) [_m/s_]  | Max Rotational Speed (_θ_) [_rad/s_]  | Radius [_m_] | Emergency-Stop¹ | Laser-range [_m_] | Holonomic² |
| :--- | :---:|  :---: |:---: |:---: |:---:|   :---:| :---:| 
| *turtlebot3-burger* | 0.22 | 0.0  | 2.84  | 0.113 | False | 3.5  | False |
| *jackal*            | 2.0  | 0.0  | 4.0  | 0.267 | False | 30.0 | False |
| *ridgeback*         | 1.1  | 0.5  | 2.0  | 0.625 | False | 10.0 | True  |
| *agvota*           | 0.5  | 0.0  | 0.4  | 0.629 | False | 5.0  | False |

For additional / more detailed information about each robot:

+ [See the parameters needed for the **Navigation stack**](https://github.com/eliastreis/arena-rosnav/tree/local_planner_subgoalmode/arena_navigation/arena_local_planner/model_based/conventional/config)
+ [See the flatland model for information like laser min/max [_rad_]](https://github.com/eliastreis/arena-rosnav/tree/local_planner_subgoalmode/simulator_setup/robot)
+ See [_HERE_](https://github.com/eliastreis/arena-rosnav/tree/local_planner_subgoalmode/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/configs) for the definition of the robts action_spaces (needed for rl-based-training)

> ___NOTE___: The _emergency-stop_ capability is currently still being development, it will however be available on all robots.

To select a robot model for your simulation run (in this case _ridgeback_):
```bash
roslaunch arena_bringup start_arena_flatland.launch model:=ridgeback
```
¹ *Stops moving when an object has been detected in the safety zone of the robot*

² *For _holonomic_ robots `vel_y = 0`; they are not able to drive directly to their left or right, but have to drive on a circular trejectory to their reach navigation goal*  
