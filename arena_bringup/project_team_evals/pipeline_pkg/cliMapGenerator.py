import numpy as np
import os
import yaml
import pathlib
from pathlib import Path
import re
from typing import List
from PIL import Image
from argparse import ArgumentParser


class MapGenerator:
    def __init__(self, args, **kwargs):
        super().__init__(**kwargs)

        self.height = int(args.height)
        self.width = int(args.width)
        self.path = args.save_path
        self.map_type = args.map_type.lower()
        self.num_obstacles = int(args.num_obstacles)
        self.num_maps = int(args.num_maps)
        self.map_res = float(args.map_res)
        self.corridor_width = int(args.corridor_width)
        self.iterations = int(args.iterations)
        self.obstacle_size = int(args.obstacle_size)

    def getMapNames(self) -> List[str]:
        """
        Generate simple map names that don't exist yet in the form of f"map{index}".
        Search the maps folder for already existing maps in this format. Get the highest index and then
        start counting from there.
        """
        folder = pathlib.Path(self.path)
        map_folders = [p for p in folder.iterdir() if p.is_dir()]
        names = [p.parts[-1] for p in map_folders]
        # get only the names that are in the form of f"map{index}"
        prefix = "map"
        pat = re.compile(f"{prefix}\d+$", flags=re.ASCII)
        filtered_names = [name for name in names if pat.match(name) != None]
        # get the max index that already exists
        max_index = 0
        if len(filtered_names) > 0:
            max_index = max([int(name[len(prefix) :]) for name in filtered_names])
        number_of_maps = self.num_maps
        # generate new names beginning with the max index
        return [f"map{i}" for i in range(max_index + 1, max_index + 1 + number_of_maps)]

    def generateMaps(self):
        # generate maps
        path = pathlib.Path(self.path)

        # create new maps with appropriate names
        map_names = self.getMapNames()
        for map_name in map_names:
            map_array = self.getCurrentMap()
            if map_array is not None:
                self.make_image(map_array, path, map_name)
                self.create_yaml_files(path / map_name)

    def getCurrentMap(self) -> np.ndarray:
        map_type = self.map_type
        height = self.height
        width = self.width

        if height < 10 or width < 10:
            return None
        map_array = None
        if map_type == "indoor":
            corridor_radius = self.corridor_width
            iterations = self.iterations
            map_array = self.create_indoor_map(height, width, corridor_radius, iterations)
        elif map_type == "outdoor":
            obstacle_number = self.num_obstacles
            obstacle_extra_radius = self.obstacle_size
            map_array = self.create_outdoor_map(
                height, width, obstacle_number, obstacle_extra_radius
            )

        return map_array

    def create_yaml_files(self, map_folder_path: pathlib.Path):
        """
        Create the files map.yaml (ROS) and map.wordl.yaml (Flatland) for the map.
        map_folder_path: path to folder for this map e.g.: /home/user/catkin_ws/src/arena-rosnav/simulator_setup/maps/mymap
        """
        map_folder = pathlib.Path(map_folder_path)
        map_name = map_folder.parts[-1]

        # create map.yaml
        map_yaml = {
            "image": "{0}.png".format(map_name),
            "resolution": self.map_res,
            "origin": [0.0, 0.0, 0.0],  # [-x,-y,0.0]
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }

        with open(str(map_folder / "map.yaml"), "w") as outfile:
            yaml.dump(map_yaml, outfile, sort_keys=False, default_flow_style=None)

        # create map.world.yaml
        world_yaml_properties = {
            "properties": {"velocity_iterations": 10, "position_iterations": 10}
        }

        world_yaml_layers = {
            "layers": [{"name": "static", "map": "map.yaml", "color": [0, 1, 0, 1]}]
        }

        with open(str(map_folder / "map.world.yaml"), "w") as outfile:
            # somehow the first part must be with default_flow_style=False
            yaml.dump(world_yaml_properties, outfile, sort_keys=False, default_flow_style=False)
            # 2nd part must be with default_flow_style=None
            yaml.dump(world_yaml_layers, outfile, sort_keys=False, default_flow_style=None)

    def make_image(self, map: np.ndarray, maps_folder_path: pathlib.Path, map_name: str):
        """
        Create PNG file from occupancy map (1:occupied, 0:free) and the necessary yaml files.
        - map: numpy array
        - maps_folder_path: path to maps folder e.g.: /home/user/catkin_ws/src/arena-rosnav/simulator_setup/maps
        - map_name: name of map, a folder will be created using this name
        """
        # create new directory for map
        map_folder = maps_folder_path / map_name
        if not map_folder.exists():
            os.mkdir(str(map_folder))
        # create image
        # monochromatic image
        img = Image.fromarray(((map - 1) ** 2 * 255).astype("uint8"))
        imgrgb = img.convert("RGB")
        # save image
        # save map in map directory
        imgrgb.save(str(map_folder / (map_name + ".png")))

    # create empty map with format given by height,width and initialize empty tree
    def initialize_map(self, height, width, type="indoor"):
        if type == "outdoor":
            map = np.tile(1, [height, width])
            map[slice(1, height - 1), slice(1, width - 1)] = 0
            return map
        else:
            return np.tile(1, [height, width])

    def insert_root_node(self, map, tree):  # create root node in center of map
        root_node = [int(np.floor(map.shape[0] / 2)), int(np.floor(map.shape[1] / 2))]
        map[root_node[0], root_node[1]] = 0
        tree.append(root_node)

    # sample position from map within boundary and leave tolerance for corridor width
    def sample(self, map, corridor_radius):
        random_x = np.random.choice(
            range(corridor_radius + 2, map.shape[0] - corridor_radius - 1, 1)
        )
        random_y = np.random.choice(
            range(corridor_radius + 2, map.shape[1] - corridor_radius - 1, 1)
        )
        return [random_x, random_y]

    # find nearest node according to L1 norm
    def find_nearest_node(self, random_position, tree):
        nearest_node = []
        min_distance = np.inf
        for node in tree:
            distance = sum(np.abs(np.array(random_position) - np.array(node)))
            if distance < min_distance:
                min_distance = distance
                nearest_node = node
        return nearest_node

    # insert new node into the map and tree
    def insert_new_node(self, random_position, tree, map):
        map[random_position[0], random_position[1]] = 0
        tree.append(random_position)

    def get_constellation(self, node1, node2):
        # there are two relevant constellations for the 2 nodes, which must be considered when creating the horizontal and vertical path
        # 1: lower left and upper right
        # 2: upper left and lower right
        # each of the 2 constellation have 2 permutations which must be considered as well
        constellation1 = {
            # x1>x2 and y1<y2
            "permutation1": node1[0] > node2[0] and node1[1] < node2[1],
            "permutation2": node1[0] < node2[0] and node1[1] > node2[1],
        }  # x1<x2 and y1>y2
        if constellation1["permutation1"] or constellation1["permutation2"]:
            return 1
        else:
            return 2

    def create_path(self, node1, node2, corridor_radius, map):
        coin_flip = np.random.random()
        # x and y coordinates must be sorted for usage with range function
        x1, x2 = sorted([node1[0], node2[0]])
        y1, y2 = sorted([node1[1], node2[1]])
        if self.get_constellation(node1, node2) == 1:  # check which constellation
            # randomly determine the curvature of the path (right turn/left turn)
            if coin_flip >= 0.5:
                map[
                    slice(x1 - corridor_radius, x1 + corridor_radius + 1),
                    range(y1 - corridor_radius, y2 + 1 + corridor_radius, 1),
                ] = 0  # horizontal path
                map[
                    range(x1 - corridor_radius, x2 + 1 + corridor_radius, 1),
                    slice(y1 - corridor_radius, y1 + corridor_radius + 1),
                ] = 0  # vertical path
            else:
                map[
                    slice(x2 - corridor_radius, x2 + corridor_radius + 1),
                    range(y1 - corridor_radius, y2 + 1 + corridor_radius, 1),
                ] = 0  # horizontal path
                map[
                    range(x1 - corridor_radius, x2 + 1 + corridor_radius, 1),
                    slice(y2 - corridor_radius, y2 + corridor_radius + 1),
                ] = 0  # vertical path
        else:
            # randomly determine the curvature of the path (right turn/left turn)
            if coin_flip >= 0.5:
                map[
                    slice(x1 - corridor_radius, x1 + corridor_radius + 1),
                    range(y1 - corridor_radius, y2 + 1 + corridor_radius, 1),
                ] = 0  # horizontal path
                map[
                    range(x1 - corridor_radius, x2 + 1 + corridor_radius, 1),
                    slice(y2 - corridor_radius, y2 + corridor_radius + 1),
                ] = 0  # vertical path
            else:
                map[
                    slice(x2 - corridor_radius, x2 + corridor_radius + 1),
                    range(y1 - corridor_radius, y2 + 1 + corridor_radius, 1),
                ] = 0  # horizontal path
                map[
                    range(x1 - corridor_radius, x2 + 1 + corridor_radius, 1),
                    slice(y1 - corridor_radius, y1 + corridor_radius + 1),
                ] = 0  # vertical path

    def create_indoor_map(self, height, width, corridor_radius, iterations):
        tree = []  # initialize empty tree
        map = self.initialize_map(height, width)
        self.insert_root_node(map, tree)
        for i in range(iterations):  # create as many paths/nodes as defined in iteration
            random_position = self.sample(map, corridor_radius)
            # nearest node must be found before inserting the new node into the tree, else nearest node will be itself
            nearest_node = self.find_nearest_node(random_position, tree)
            self.insert_new_node(random_position, tree, map)
            self.create_path(random_position, nearest_node, corridor_radius, map)
        return map

    def create_outdoor_map(self, height, width, obstacle_number, obstacle_extra_radius):
        map = self.initialize_map(height, width, type="outdoor")
        for i in range(obstacle_number):
            random_position = self.sample(map, obstacle_extra_radius)
            map[
                slice(
                    random_position[0] - obstacle_extra_radius,
                    random_position[0] + obstacle_extra_radius + 1,
                ),  # create 1 pixel obstacles with extra radius if specified
                slice(
                    random_position[1] - obstacle_extra_radius,
                    random_position[1] + obstacle_extra_radius + 1,
                ),
            ] = 1
        return map


if __name__ == "__main__":

    dirname = os.path.dirname(__file__)
    output_path = Path(dirname) / "maps"

    # create dir if not exists
    output_path.mkdir(parents=True, exist_ok=True)
    default_save_path = str(output_path)

    parser = ArgumentParser()
    parser.add_argument(
        "--width",
        action="store",
        dest="width",
        default=100,
        help="Width of the map",
        required=False,
    )
    parser.add_argument(
        "--height",
        action="store",
        dest="height",
        default=100,
        help="Width of the map",
        required=False,
    )
    parser.add_argument(
        "--map_type",
        action="store",
        dest="map_type",
        default="indoor",
        help="Width of the map",
        required=False,
    )
    parser.add_argument(
        "--num_maps",
        action="store",
        dest="num_maps",
        default=10,
        help="Width of the map",
        required=False,
    )
    parser.add_argument(
        "--map_res",
        action="store",
        dest="map_res",
        default=0.5,
        help="Width of the map",
        required=False,
    )
    parser.add_argument(
        "--save_path",
        action="store",
        dest="save_path",
        default=default_save_path,
        help="Width of the map",
        required=False,
    )
    parser.add_argument(
        "--iterations",
        action="store",
        dest="iterations",
        default=50,
        help="Width of the map",
        required=False,
    )
    parser.add_argument(
        "--num_obstacles",
        action="store",
        dest="num_obstacles",
        default=30,
        help="Width of the map",
        required=False,
    )
    parser.add_argument(
        "--obstacle_size",
        action="store",
        dest="obstacle_size",
        default=5,
        help="Width of the map",
        required=False,
    )
    parser.add_argument(
        "--corridor_width",
        action="store",
        dest="corridor_width",
        default=3,
        help="Width of the map",
        required=False,
    )

    args = parser.parse_args()
    map_gen = MapGenerator(args)

    map_gen.generateMaps()
