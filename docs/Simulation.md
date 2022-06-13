## Simulation

# Robots

Arena rosnav incluedes five different robots.

- [_turtlebot3-burger_](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)
- [_jackal_](https://www.clearpathrobotics.com/assets/guides/melodic/jackal/simulation.html)
- [_ridgeback_](https://www.clearpathrobotics.com/assets/guides/melodic/ridgeback/simulation.html)
- [_agvota_](https://github.com/inomuh/agv)
- [_dingo_](https://www.clearpathrobotics.com/assets/guides/melodic/dingo/simulation.html)

All robots are equipt with a laser scanner. The robots differ in size, laser-range etc. See below tabel for more detailed information on each robot:

| Name                | Max Speed (v*x) [\_m/s*] | Max Speed (v*y) [\_m/s*] | Max Rotational Speed (_θ_) [_rad/s_] | Radius [_m_] | Emergency-Stop¹ | Laser-range [_m_] | Holonomic² |
| :------------------ | :----------------------: | :----------------------: | :----------------------------------: | :----------: | :-------------: | :---------------: | :--------: |
| _turtlebot3-burger_ |           0.22           |           0.0            |                 2.84                 |    0.113     |      False      |        3.5        |   False    |
| _jackal_            |           2.0            |           0.0            |                 4.0                  |    0.267     |      False      |       30.0        |   False    |
| _ridgeback_         |           1.1            |           0.5            |                 2.0                  |    0.625     |      False      |       10.0        |    True    |
| _agvota_            |           0.5            |           0.0            |                 0.4                  |    0.629     |      False      |        5.0        |   False    |
| _dingo_             |           1.3            |           0.0            |                 4.0                  |    0.278     |      False      |       30.0        |   False    |

For additional / more detailed information about each robot:

- [See the parameters needed for the **Navigation stack**](https://github.com/eliastreis/arena-rosnav/tree/local_planner_subgoalmode/arena_navigation/arena_local_planner/model_based/conventional/config)
- [See the flatland model for information like laser min/max [_rad_]](https://github.com/eliastreis/arena-rosnav/tree/local_planner_subgoalmode/simulator_setup/robot)
- See [_HERE_](https://github.com/eliastreis/arena-rosnav/tree/local_planner_subgoalmode/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/configs) for the definition of the robts action_spaces (needed for rl-based-training)

> **_NOTE_**: The _emergency-stop_ capability is currently still being development, it will however be available on all robots.

To select a robot model for your simulation run (in this case _ridgeback_):

```bash
roslaunch arena_bringup start_arena_flatland.launch model:=ridgeback
```

¹ _Stops moving when an object has been detected in the safety zone of the robot_

² _For *holonomic* robots `vel_y = 0`; they are not able to drive directly to their left or right, but have to drive on a circular trejectory to their reach navigation goal_
