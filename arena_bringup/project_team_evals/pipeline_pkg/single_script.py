import os
from pathlib import Path

dirname = os.path.dirname(__file__)

# Generate maps
maps_path = Path(dirname) / "maps"

# create dir if not exists
maps_path.mkdir(parents=True, exist_ok=True)
maps_path = str(maps_path)


width = 100
height = 100
map_type = "indoor"
num_maps = 3
map_res = 0.5
maps_path = "./maps"
iterations = 50
num_obstacles = 30
obstacle_size = 5
corridor_width = 3

generate_maps_command = f"python3 cliMapGenerator.py --width {width} --height {height} --map_type {map_type} --num_maps {num_maps} --map_res {map_res} --save_path {maps_path} --iterations {iterations} --num_obstacles {num_obstacles} --obstacle_size {obstacle_size} --corridor_width {corridor_width}"
os.system(generate_maps_command)

#----------------------------------------------------------

# Run simulations and record data
local_planners = ["dwa"]
robot_models = ["burger"]
generated_maps = os.listdir(maps_path)
num_dynamic_obs = ["5"]
num_static_obs = ["3"]

dyn_obs_velocity = ("0.1", "0.5")
dyn_obs_radius = ("0.2", "0.5")
static_obs_vertices = ("3", "8")

for planner in local_planners:
    for robot in robot_models:
        for num_dyn in num_dynamic_obs:
            for num_static in num_static_obs:
                for gen_map in generated_maps:
                    roslaunch_command = f""" roslaunch arena_bringup start_arena_flatland.launch model:={robot} num_dynamic_obs:={num_dyn} num_static_obs:={num_static} min_dyn_vel:={dyn_obs_velocity[0]} max_dyn_vel:={dyn_obs_velocity[1]} min_dyn_radius:={dyn_obs_radius[0]} max_dyn_radius:={dyn_obs_radius[1]} min_static_num_vertices:={static_obs_vertices[0]} max_static_num_vertices:={static_obs_vertices[1]} local_planner:={planner} map_file:={gen_map} task_mode:="project_eval" scenario_file:="project_eval/scenario_1.json" use_recorder:="true" data_records_path:={maps_path} show_rviz:="false" use_rviz:="false" """
                    os.system(roslaunch_command)

#----------------------------------------------------------

# Data cleaning, analysis and map complexity calculation
data_records_path = Path(dirname) / "sims_data_records"

# create dir if not exists
data_records_path.mkdir(parents=True, exist_ok=True)
data_records_path = str(data_records_path)

avg_command = f"python3 createAverage.py --image_path {maps_path} --csv_path {data_records_path}"

#----------------------------------------------------------