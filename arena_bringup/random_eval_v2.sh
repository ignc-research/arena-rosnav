local_planners=("rosnav" "mpc" "dwa" "teb" "rlca")
robot_models=("burger" "jackal" "ridgeback" "agvota")

min_dyn_vel="0.8"
max_dyn_vel="1.5"
min_dyn_radius="0.5"
max_dyn_radius="1.5"
min_static_radius="0.5"
max_static_radius="2"

for planner in "${local_planners[@]}"; 
do for robot in "${robot_models}";
do roslaunch arena_bringup start_arena_flatland.launch model:="$robot" num_dynamic_obs:=$1 num_static_obs:=$2 min_dyn_vel:=$min_dyn_vel max_dyn_vel:=$max_dyn_vel min_dyn_radius:=$min_dyn_radius max_dyn_radius:=$max_dyn_radius min_static_radius:=$min_static_radius max_static_radius:=$max_static_radius local_planner:="$planner" map_file:="random_map" task_mode:="random_eval" scenario_file:="random_eval/random_indoor_project_scenario.json" use_recorder:="true"; 
done; done
