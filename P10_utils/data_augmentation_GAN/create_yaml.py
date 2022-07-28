import os
from argparse import ArgumentParser
import yaml
import glob
import shutil


class Create_yml:
    def create_yaml_files(self, name, dir_path, resolution):

        map_name = name.split(".")
        map_yaml = {
            "image": "{0}.png".format(map_name[0]),
            "resolution": resolution,
            "origin": [0.0, 0.0, 0.0],  # [-x,-y,0.0]
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }
        # dir_path = os.path.dirname(os.path.realpath(__file__)) # get path for current file, does not work if os.chdir() was used, NOTE: change directory if needed

        try:
            os.mkdir(
                dir_path + "/" + map_name[0]
            )  # create directory based on mapname where this script is located
        except:
            pass
        new_path = dir_path + "/" + map_name[0]
        with open(new_path + "/map.yaml", "w") as outfile:
            yaml.dump(map_yaml, outfile, sort_keys=False, default_flow_style=None)

        world_yaml_properties = {
            "properties": {"velocity_iterations": 10, "position_iterations": 10}
        }
        world_yaml_layers = {
            "layers": [{"name": "static", "map": "map.yaml", "color": [0, 1, 0, 1]}]
        }

        with open(new_path + "/map.world.yaml", "w") as outfile:
            yaml.dump(
                world_yaml_properties,
                outfile,
                sort_keys=False,
                default_flow_style=False,
            )  # somehow the first part must be with default_flow_style=False
            yaml.dump(
                world_yaml_layers, outfile, sort_keys=False, default_flow_style=None
            )  # 2nd part must be with default_flow_style=None

        shutil.move(dir_path + "/" + name, new_path)


def main():
    parser = ArgumentParser()
    parser.add_argument(
        "--image_path",
        action="store",
        dest="image_path",
        help="path to the floor plan of your world. (in .pgm , .jpg or .png format)",
        required=True,
    )
    args = parser.parse_args()
    path = os.path.join(args.image_path, "*.png")
    inputs = glob.glob(path)
    res = 0.2
    for img in inputs:
        head, tail = os.path.split(img)
        Create_yml().create_yaml_files(tail, head, res)
    print("-------------------------------------------------------------------------")
    print("Done")


if __name__ == "__main__":
    main()
