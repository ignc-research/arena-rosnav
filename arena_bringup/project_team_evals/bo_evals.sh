local_planners=("rosnav")
robot_models=("burger")
project_maps=("map_75" "map_76" "map_77" "map_78" "map_79")

num_dynamic_obs="20"
num_static_obs="5"
min_dyn_vel="0.2"
max_dyn_vel="0.8"
min_dyn_radius="0.2"
max_dyn_radius="0.5"
min_static_radius="0.5"
max_static_radius="1.5"

for planner in "${local_planners[@]}"; 
do for robot in "${robot_models[@]}";
do for map in "${project_maps[@]}";
do roslaunch arena_bringup start_arena_flatland.launch model:="$robot" num_dynamic_obs:=$num_dynamic_obs num_static_obs:=$num_static_obs min_dyn_vel:=$min_dyn_vel max_dyn_vel:=$max_dyn_vel min_dyn_radius:=$min_dyn_radius max_dyn_radius:=$max_dyn_radius min_static_radius:=$min_static_radius max_static_radius:=$max_static_radius local_planner:="$planner" map_file:="$map" task_mode:="project_eval" scenario_file:="random_eval/random_indoor_project_scenario.json" use_recorder:="true"; 
done; done; done