import os

# Generate maps
os.system("python3 MapGenerator.py")


local_planners = ["dwa"]
robot_models = ["burger"]
# TODO: Automatically generated map
project_maps = []
num_dynamic_obs = ["5"]
num_static_obs = ["3"]

dyn_obs_velocity = ("0.1", "0.5")
dyn_obs_radius = ("0.2", "0.5")
static_obs_vertices = ("3", "8")

for planner in local_planners:
    for robot in robot_models:
        for num_dyn in num_dynamic_obs:
            for num_static in num_static_obs:
                command = f""" roslaunch arena_bringup start_arena_flatland.launch model:={robot} num_dynamic_obs:={num_dyn} num_static_obs:={num_static} min_dyn_vel:={dyn_obs_velocity[0]} max_dyn_vel:={dyn_obs_velocity[1]} min_dyn_radius:={dyn_obs_radius[0]} max_dyn_radius:={dyn_obs_radius[1]} min_static_num_vertices:={static_obs_vertices[0]} max_static_num_vertices:={static_obs_vertices[1]} local_planner:={planner} map_file:="map10" task_mode:="project_eval" scenario_file:="project_eval/scenario_1.json" use_recorder:="true" show_rviz:="false" use_rviz:="false" """
                os.system(command)
