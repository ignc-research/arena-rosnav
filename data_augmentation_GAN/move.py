
from argparse import ArgumentParser
from pathlib import Path
from tqdm import tqdm
import os
import shutil

parser = ArgumentParser()
parser.add_argument(
        "--folders_path",
        action="store",
        dest="folders_path",
        default=f"/home/elias/catkin_ws/src/arena-rosnav-3D/simulator_setup/maps/ignc/map.pgm",
        help="path to the floor plan of your world. Usually in .pgm format",
        required=False,
    )
parser.add_argument(
    "--image_path",
    action="store",
    dest="image_path",
    default=f"/home/elias/catkin_ws/src/arena-rosnav-3D/simulator_setup/maps/ignc/map.yaml",
    help="path to the .yaml description file of your floor plan",
    required=False,
)

args = parser.parse_args()
converted_dict = vars(args)
file_name = Path(converted_dict["image_path"]).stem

    # extract data
if args.folders_path:
    dirs = [x[0] for x in os.walk(args.folders_path)][1:]
    for dir in tqdm(dirs):
        file_list=os.listdir(dir)
        matching = [s for s in file_list if ".png" in s]
        shutil.move(dir + '/'+matching[0], args.image_path)

