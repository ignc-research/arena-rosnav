import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
import random


def generate_freespace_indices(map_: OccupancyGrid) -> tuple:
    """generate the indices(represented in a tuple) of the freesapce based on the map

    Returns:
        indices_y_x(tuple): indices of the non-occupied cells, the first element is the y-axis indices,
        the second element is the x-axis indices.
    """
    width_in_cell, height_in_cell = map_.info.width, map_.info.height
    map_2d = np.reshape(map_.data, (height_in_cell, width_in_cell))
    indices_y_x = np.where(map_2d == 0)
    return indices_y_x


def get_random_pos_on_map(free_space_indices, map_: OccupancyGrid, safe_dist: float, forbidden_zones: list = None):
    """
    Args:
        indices_y_x(tuple): a 2 elementary tuple stores the indices of the non-occupied cells, the first element is the y-axis indices,
            the second element is the x-axis indices.
        map (OccupancyGrid): map proviced by the ros map service
        forbidden_zones (list of 3 elementary tuple(x,y,r)): a list of zones which is forbidden
    Returns:
       x_in_meters,y_in_meters,theta
    """

    def is_pos_valid(x_in_meters, y_in_meters):
        for forbidden_zone in forbidden_zones:
            if (x_in_meters-forbidden_zone[0])**2+(y_in_meters-forbidden_zone[1])**2 < (forbidden_zone[2]+safe_dist)**2:
                return False

        # in pixel
        cell_radius = int(safe_dist / map_.info.resolution)
        x_index = int((x_in_meters - map_.info.origin.position.x) // map_.info.resolution)
        y_index = int((y_in_meters - map_.info.origin.position.y) // map_.info.resolution)

        # check occupancy around (x_index,y_index) with cell_radius
        # TODO use numpy for checking
        for i in range(x_index - cell_radius, x_index + cell_radius, 1):
            for j in range(y_index - cell_radius, y_index + cell_radius, 1):
                index = j * map_.info.width + i
                if index >= len(map_.data):
                    return False
                try:
                    value = map_.data[index]
                except IndexError:
                    print("IndexError: index: %d, map_length: %d" %
                          (index, len(map_.data)))
                    return False
                if value != 0:

                    return False
        return True

    assert len(free_space_indices) == 2 and len(free_space_indices[0]) == len(
        free_space_indices[1]), "free_space_indices is not correctly setup"
    if forbidden_zones is None:
        forbidden_zones = []

    n_freespace_cells = len(free_space_indices[0])
    pos_valid = False
    n_check_failed = 0
    x_in_meters, y_in_meters = None, None
    while not pos_valid:
        idx = random.randint(0, n_freespace_cells-1)
        # in cells
        y_in_cells, x_in_cells = free_space_indices[0][idx], free_space_indices[1][idx]
        # convert x, y in meters
        y_in_meters = y_in_cells * map_.info.resolution + map_.info.origin.position.y
        x_in_meters = x_in_cells * map_.info.resolution + map_.info.origin.position.x
        pos_valid = is_pos_valid(x_in_meters, y_in_meters)
        if not pos_valid:
            n_check_failed += 1
            if n_check_failed > 100:
                raise Exception(
                    "cann't find any no-occupied space please check the map information")
        # in radius
    theta = random.uniform(-math.pi, math.pi)

    return x_in_meters, y_in_meters, theta


def update_freespace_indices(free_space_indices, map_: OccupancyGrid, vertexArray) -> tuple:
    """update the indices(represented in a tuple) of the freespace based on the map and the static polygons
    ostacles manuelly added 
    param map_ : original occupacy grid
    param vertlist: vertex of the polygons

    Returns:
        indices_y_x(tuple): indices of the non-occupied cells, the first element is the y-axis indices,
        the second element is the x-axis indices.
    """
    # free_space_indices=generate_freespace_indices(map_)
    print(vertexArray)
    n_freespace_cells = len(free_space_indices[0])
    mask=[]
    for idx in range(n_freespace_cells):
        # in cells
        y_in_cells, x_in_cells = free_space_indices[0][idx], free_space_indices[1][idx]
        # convert x, y in meters
        y_in_meters = y_in_cells * map_.info.resolution + map_.info.origin.position.y
        x_in_meters = x_in_cells * map_.info.resolution + map_.info.origin.position.x
        p=np.array([x_in_meters,y_in_meters])
        size=vertexArray.shape[0]
        #check if the point is in the polygon or not
        for i in range(size):
            v1=vertexArray[i]-p
            v1=v1/np.linalg.norm(v1)
            v2=vertexArray[(i+1)%size]-p
            v2=v2/np.linalg.norm(v2)
            c1=np.cross(v1,v2)
            v3=vertexArray[(i+2)%size]-p
            v3=v3/np.linalg.norm(v3)
            c2=np.cross(v2,v3)
            if c1*c2 <0:                
                mask.append(True)
                break
        else:
            print(p)
            mask.append(False)
    free_space_indices_new=(free_space_indices[0][mask],free_space_indices[0][mask])
    return free_space_indices_new


def generate_map_inner_border(free_space_indices, map_: OccupancyGrid):
    """generate map border (four vertices of the map)

    Returns:
        vertex_coordinate_x_y(np.ndarray with shape 4 x 2):
    """
    n_freespace_cells = len(free_space_indices[0])
    border_vertex=np.array([]).reshape(0,2)
    border_vertices=np.array([]).reshape(0,2)
    for idx in [0,n_freespace_cells-1]:
        y_in_cells, x_in_cells = free_space_indices[0][idx], free_space_indices[1][idx]
        y_in_meters = y_in_cells * map_.info.resolution + map_.info.origin.position.y
        x_in_meters = x_in_cells * map_.info.resolution + map_.info.origin.position.x
        border_vertex=np.vstack([border_vertex, [x_in_meters, y_in_meters]])
    border_vertices=np.vstack([border_vertices, [border_vertex[0,0],border_vertex[0,1]]])
    border_vertices=np.vstack([border_vertices, [border_vertex[0,0],border_vertex[1,1]]])
    border_vertices=np.vstack([border_vertices, [border_vertex[1,0],border_vertex[0,1]]])
    border_vertices=np.vstack([border_vertices, [border_vertex[1,0],border_vertex[1,1]]])
    return border_vertices
