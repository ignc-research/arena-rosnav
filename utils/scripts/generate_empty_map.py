from PIL import Image
from pathlib import Path
import yaml


def generate_empty_map(map_parent_dir_path, map_dir_name, width_in_meter, height_in_meter, resolution):

    map_dir_path = Path(map_parent_dir_path, map_dir_name)
    try:
        map_dir_path.mkdir(parents=True)
    except OSError:
        user_input = input("The folder \"{map_dir_path}\" is alredy existing, do you want to overwrite the files inside? y[es] or n[o]?\t")
        if user_input.startswith('n'):
            exit(0)
    # generate map png file
    w_in_pixel = int(width_in_meter/resolution)
    h_in_pixel = int(height_in_meter/resolution)
    w_edge = max(int(w_in_pixel/40), 5)
    white_layer = Image.new('L', (w_in_pixel, h_in_pixel), 255)
    map_image = Image.new('L', (w_in_pixel+2*w_edge, h_in_pixel+2*w_edge))
    map_image.paste(white_layer, (w_edge, w_edge))
    map_image_name = 'empty_map.png'
    map_image.save(map_dir_path.joinpath(map_image_name))

    # generate map yaml file
    map_yaml_template = """
                        image: map_small.png
                        resolution: 0.05
                        origin: [-6.0, -6.0, 0.0]
                        occupied_thresh: 0.65
                        free_thresh: 0.196
                        negate: 0
                        """
    map_yaml_data = yaml.safe_load(map_yaml_template)
    map_yaml_data['image'] = map_image_name
    map_yaml_data['resolution'] = resolution
    yaml.safe_dump(map_yaml_data, map_dir_path.joinpath('map.yaml').open('w'))

    # generate map.world yaml file, use yaml paser to format it
    map_world_template = """
                        properties: 
                            velocity_iterations: 10

                            position_iterations: 10
                            layers:  # Support for arbitrary number of layers
                            - name: "static"
                            map: "map.yaml"  # leading / denotes absolute file path, otherwise relative
                            color: [0, 1, 0, 1]  # List of floats [r,g,b,a] to color debug boundary
                        """
    map_world_data = yaml.safe_load(map_world_template)
    yaml.safe_dump(map_world_data, map_dir_path.joinpath(
        'map.world.yaml').open('w'))
    print('new map has been created at: {map_dir_path.} !')


if __name__ == "__main__":
    import argparse
    argparser = argparse.ArgumentParser()
    argparser.add_argument('-p', "--parent_dir_path",
                           help="The path of parent directory", default='simulator_setup/maps')
    argparser.add_argument(
        '-n', '--map_dir_name', help='The path of the map directory', default='small_empty_map')
    argparser.add_argument(
        '--width', help='with in meter', default=2, type=int)
    argparser.add_argument(
        '--height', help='height in meter', default=2, type=int)
    argparser.add_argument(
        '-r', '--resolution', help='resolution meter/pixel', default=0.05, type=float)
    args = argparser.parse_args()
    generate_empty_map(args.parent_dir_path, args.map_dir_name,
                       args.width, args.height, args.resolution)
