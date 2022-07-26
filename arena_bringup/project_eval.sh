local_planners=("rosnav" "mpc" "dwa" "teb" "rlca")
robot_models=("burger" "jackal" "ridgeback" "agvota")
project_maps=`ls ../simulator_setup/maps | grep -m 100 "map"`

min_dyn_vel="0.2"
max_dyn_vel="0.8"
min_dyn_radius="0.2"
max_dyn_radius="0.4"
min_static_radius="0.5"
max_static_radius="2"


for planner in "${local_planners[@]}"; 
do for robot in "${robot_models[@]}";
do for map in "${project_maps[@]}";
do roslaunch arena_bringup start_arena_flatland.launch model:="$robot" num_dynamic_obs:=$1 num_static_obs:=$2 min_dyn_vel:=$min_dyn_vel max_dyn_vel:=$max_dyn_vel min_dyn_radius:=$min_dyn_radius max_dyn_radius:=$max_dyn_radius min_static_radius:=$min_static_radius max_static_radius:=$max_static_radius local_planner:="$planner" map_file:="$map" task_mode:="project_eval" scenario_file:="project_eval/scenario_1.json" use_recorder:="true" show_rviz:="false" use_rviz:="false"; 
done; done; done
