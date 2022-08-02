local_planners=("dwa")
robot_models=("burger")
project_maps=("map1")
num_dynamic_obs=("5")

num_static_obs="5"
min_dyn_vel="0.1"
max_dyn_vel="0.5"
min_dyn_radius="0.2"
max_dyn_radius="0.5"
min_static_num_vertices="3"
# Max possible value without errors = 8
max_static_num_vertices="8"

for planner in "${local_planners[@]}"; 
do for robot in "${robot_models[@]}";
do for map in "${project_maps[@]}";
do for num_dyn in "${num_dynamic_obs[@]}"
do roslaunch arena_bringup start_arena_flatland.launch model:="$robot" num_dynamic_obs:="$num_dyn" num_static_obs:=$num_static_obs min_dyn_vel:=$min_dyn_vel max_dyn_vel:=$max_dyn_vel min_dyn_radius:=$min_dyn_radius max_dyn_radius:=$max_dyn_radius min_static_num_vertices:=$min_static_num_vertices max_static_num_vertices:=$max_static_num_vertices local_planner:="$planner" map_file:="$map" task_mode:="project_eval" scenario_file:="project_eval/scenario_1.json" show_rviz:="false" use_rviz:="false" use_recorder:="true"; 
done; done; done; done