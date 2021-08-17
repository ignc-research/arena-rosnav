import pathlib

import rospkg
import rospy
import ctypes


def start_teb_nodes(ns: str, path_to_config: str):
    with open(paths['all_in_one_parameters'], 'r') as model_json:
        config_data = json.load(model_json)

    assert config_data is not None, "Error: All in one parameter file cannot be found!"
    assert "teb" in config_data['models'].keys(), "Error: No teb config given in all in one parameter file!"

    teb_data = config_data['models']['teb']

    teb_names = teb_data['teb_names']
    teb_params = teb_data['teb_params']
    local_costmap_params = teb_data['local_costmap_params']

    for i in range(len(teb_names)):
        # set teb parameters
        _set_teb_parameters(ns + "/" + teb_names[i], teb_params[i])

        # add frame parameters
        local_costmap_params_i: dict = local_costmap_params[i]
        local_costmap_params_i["scan/sensor_frame"] = ns + "_laser_link"
        local_costmap_params_i["global_frame"] = ns + "_odom"
        local_costmap_params_i["robot_base_frame"] = ns + "_base_footprint"

        # set local costmap parameters
        _set_up_local_costmap_parameters(ns, teb_names[i] + "/" + "local_costmap", local_costmap_params_i)

        # start teb node



def _set_teb_parameters(ns: str, params: dict):
    for key, value in enumerate(params):
        rospy.set_param(ns + "/" + key, value)


def _set_up_local_costmap_parameters(ns:str, suffix: str, params: dict):
    for key, value in enumerate(params):
        rospy.set_param(ns + "/" + suffix + "/" + key, value)

    # rename clock topic
    rospy.remap_name("clock", ns + "/" + "clock")
    rospy.remap_name("map", "/map")

def _start_teb_node(ns: str, name: str):
    libname = "liball_in_one_teb_interface.so"
    c_lib = ctypes.CDLL(libname)
    c_lib.start_teb_all_in_one_node("test", "test")
