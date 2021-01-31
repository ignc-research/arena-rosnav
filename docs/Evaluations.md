## Evaluations
We provide tools to evaluate the planners. 

### Usage
1) ``` roslaunch arena_bringup start_arena_flatland.launch disable_scenario:="false" map_file:="map1" scenario_file:="eval/obstacle_map1_obs10.json" local_planner:="teb" ```

  explanation:
    * disable_scenario:="false": to start a scenario (otherwise only the map will be loaded without additional obstacles / goals)
    * map_file:="map1": select a map file (ie map_file:="map_empty") for the map without static obstacles
    * scenario_file:="eval/obstacle_map1_obs10.json": select the corresponding scenario file
    * local_planner:="teb": choose between "dwa", "teb", "mpc"
    * in order to change the velocity of the dynamic obstacles, the scenario file must be edited
    
2) ``` rosbag record -o cadrl_map1_ob10_vel_01 /scenario_reset -e "(.*)police(.*)" ```
  explanation:
    * this command will record all topics necessary for evaluation
    * -e "(.*)police(.*)": records all topics containing "police"
    * cadrl_map1_ob10_vel_01: name of bag file, in this example the cadrl planner was recorded in map1 with 10 obstacles with lin_vel = 0.1
    
