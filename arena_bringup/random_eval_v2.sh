local_planners=("rosnav" "mpc" "dwa" "teb" "rlca")
#robots=("turtlebot3-burger" "jackal" "ridgeback" "agvota")

for planner in "${local_planners[@]}"; 
do roslaunch arena_bringup start_arena_flatland.launch num_dynamic_obs:=$1 num_static_obs:=$2 local_planner:="$planner" map_file:="random_map" task_mode:="random_eval" scenario_file:="random_eval/random_indoor_project_scenario.json" use_recorder:="true"; done
