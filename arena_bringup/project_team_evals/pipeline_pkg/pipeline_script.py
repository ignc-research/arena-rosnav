import os
import glob
from uuid import uuid4 as uuid
from pathlib import Path
from argparse import ArgumentParser
from random import randint, choice
import cv2

# Create and parse cli arguments #------------------

parser = ArgumentParser()
parser.add_argument(
    "--num_maps",
    action="store",
    dest="num_maps",
    default=10,
    help="How many maps do you want to create",
    required=False,
)

parser.add_argument(
    "--num_settings",
    action="store",
    dest="num_settings",
    default=1,
    help="How many different simulation settings you want to run on each map",
    required=False,
)
args = parser.parse_args()

num_maps = int(args.num_maps)
num_settings = int(args.num_settings)

#---------------------------------------------------
# Create necessary directories #--------------------

dirname = os.path.dirname(__file__)

# Create local maps folder if it does not exist
local_maps = Path(dirname) / "maps"
local_maps.mkdir(parents=True, exist_ok=True)

# Create local records folder if it does not exist
local_records = Path(dirname) / "sims_data_records"
local_records.mkdir(parents=True, exist_ok=True)

#---------------------------------------------------
# Set arena-rosnav paths ---------------------------

maps_path = "../../../simulator_setup/maps"
records_path = "../../../../forks/arena-evaluation/01_recording/project_recordings"

#---------------------------------------------------
# Pipeline loop #-----------------------------------

for i in range(num_maps):
    
    # Generate maps #-----------------------------------------

    map_name = str(uuid())
    width = randint(80, 150)
    height = randint(80, 150)
    map_type = choice(["indoor", "outdoor"])
    num_maps_to_generate = 1
    map_res = 0.5
    iterations = randint(80, 200)
    num_obstacles = randint(30, 60)
    obstacle_size = 3
    corridor_width = 3

    generate_maps_command = f"python3 cliMapGenerator.py --map_name {map_name} --width {width} --height {height} --map_type {map_type} --num_maps {num_maps_to_generate} --map_res {map_res} --save_path {maps_path} --iterations {iterations} --num_obstacles {num_obstacles} --obstacle_size {obstacle_size} --corridor_width {corridor_width}"
    os.system(generate_maps_command)
    
    #---------------------------------------------------------
    # Add padding to map image to 150x150 pixels #------------
    
    image_path = f"{maps_path}/{map_name}/{map_name}.png"
    img_file = cv2.imread(image_path)
    
    width_padding = 150 - width
    height_padding = 150 - height
    image = cv2.copyMakeBorder(img_file, height_padding, 0, width_padding, 0, cv2.BORDER_CONSTANT)
    
    cv2.imwrite(image_path, image)
    
    #---------------------------------------------------------
    # Run simulations and record data #-----------------------

    local_planners = ["dwa"]
    robot_models = ["burger"]    
    dyn_obs_velocity = (0.1, 0.5)
    dyn_obs_radius = (0.2, 0.5)
    static_obs_vertices = (3, 8)
    obstacles_settings = []
    
    for j in range(num_settings):
        obstacles_settings.append((randint(0, 15), randint(0, 15)))
    
    for planner in local_planners:
        for robot in robot_models:
            for sett in obstacles_settings:
                num_dyn_obs = sett[0]
                num_static_obs = sett[1]
                roslaunch_command = f""" roslaunch arena_bringup start_arena_flatland.launch model:={robot} num_dynamic_obs:={num_dyn_obs} num_static_obs:={num_static_obs} min_dyn_vel:={dyn_obs_velocity[0]} max_dyn_vel:={dyn_obs_velocity[1]} min_dyn_radius:={dyn_obs_radius[0]} max_dyn_radius:={dyn_obs_radius[1]} min_static_num_vertices:={static_obs_vertices[0]} max_static_num_vertices:={static_obs_vertices[1]} local_planner:={planner} map_file:={map_name} task_mode:="project_eval" scenario_file:="project_eval/scenario_1.json" use_recorder:="true" show_rviz:="false" use_rviz:="false" """
                os.system(roslaunch_command)


    # Copy new generated map to local maps folder
    os.system(f"mv {maps_path}/{map_name} maps")
    
    # Copy recorded data for the new map to local sims_data_records folder
    os.system(f"mv {records_path}/{map_name} sims_data_records")
        
    #---------------------------------------------------------
    # Data cleaning, analysis and map complexity calculation #
    os.system("python3 createAverage.py --csv_name /{}/{}*.csv".format(map_name,map_name))
    
    #----------------------------------------------------------
