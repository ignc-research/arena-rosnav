import os


# Generate maps
width = 100
height = 100
map_type = "indoor"
num_maps = 10
map_res = 0.5
save_path = "./maps"
iterations = 50
num_obstacles = 30
obstacle_size = 5
corridor_width = 3

generate_maps_command = f"python3 MapGenerator.py --width {width} --height {height} --map_type {map_type} --num_maps {num_maps} --map_res {map_res} --save_path {save_path} --iterations {iterations} --num_obstacles {num_obstacles} --obstacle_size {obstacle_size} --corridor_width {corridor_width}"
os.system(generate_maps_command)

#---------------------------------------------------

# Run simulations and record data
local_planners = ["dwa"]
robot_models = ["burger"]
# TODO: Automatically generated map
project_maps = os.listdir(save_path)
num_dynamic_obs = ["5"]
num_static_obs = ["3"]

dyn_obs_velocity = ("0.1", "0.5")
dyn_obs_radius = ("0.2", "0.5")
static_obs_vertices = ("3", "8")

for planner in local_planners:
    for robot in robot_models:
        for num_dyn in num_dynamic_obs:
            for num_static in num_static_obs:
                roslaunch_command = f""" roslaunch arena_bringup start_arena_flatland.launch model:={robot} num_dynamic_obs:={num_dyn} num_static_obs:={num_static} min_dyn_vel:={dyn_obs_velocity[0]} max_dyn_vel:={dyn_obs_velocity[1]} min_dyn_radius:={dyn_obs_radius[0]} max_dyn_radius:={dyn_obs_radius[1]} min_static_num_vertices:={static_obs_vertices[0]} max_static_num_vertices:={static_obs_vertices[1]} local_planner:={planner} map_file:="map10" task_mode:="project_eval" scenario_file:="project_eval/scenario_1.json" use_recorder:="true" show_rviz:="false" use_rviz:="false" """
                os.system(roslaunch_command)
