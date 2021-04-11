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
            # print(p)
            mask.append(False)
    free_space_indices_new=(free_space_indices[0][mask],free_space_indices[0][mask])
    return free_space_indices_new


def generate_map_inner_border(free_space_indices, map_: OccupancyGrid):
    """generate map border (four vertices of the map)

    Returns:
        vertex_coordinate_x_y(np.ndarray with shape 4 x 2):
    """
    n_freespace_cells = len(free_space_indices[0])
    border_vertex=np.array([]).reshape(0, 2)
    border_vertices=np.array([]).reshape(0, 2)
    for idx in [0, n_freespace_cells-4]:
        y_in_cells, x_in_cells = free_space_indices[0][idx], free_space_indices[1][idx]
        y_in_meters = y_in_cells * map_.info.resolution + map_.info.origin.position.y
        x_in_meters = x_in_cells * map_.info.resolution + map_.info.origin.position.x
        border_vertex=np.vstack([border_vertex, [x_in_meters, y_in_meters]])
    border_vertices=np.vstack([border_vertices, [border_vertex[0,0],border_vertex[0,1]]])
    border_vertices=np.vstack([border_vertices, [border_vertex[0,0],border_vertex[1,1]]])
    border_vertices=np.vstack([border_vertices, [border_vertex[1,0],border_vertex[1,1]]])
    border_vertices=np.vstack([border_vertices, [border_vertex[1,0],border_vertex[0,1]]])
    # print('border',border_vertices)
    return border_vertices

class Maze(object):
    def __init__(self):
        self.mapInnerBorderLength=None
        self.wallWidth=0.2

    def build_maze(self, free_space_indices, map_: OccupancyGrid):
        """generate maze stripes on the map

        Returns:
            vertex_coordinates_x_y(np.ndarray with shape #(one long wall + one short wall)x 4)
            short_wall_centers_coordinates_x_y(np.ndarray with shape #number of walls x 4)
            long_wall_centers_coordinates_x_y(np.ndarray with shape #number of walls x 4)
            new_free_space_indices
        """
        n_freespace_cells = len(free_space_indices[0])

        border_vertex=np.array([]).reshape(0, 2)
        border_vertices=np.array([]).reshape(0, 2)

        #calculate map border
        for idx in [0, n_freespace_cells-4]:
            y_in_cells, x_in_cells = free_space_indices[0][idx], free_space_indices[1][idx]
            y_in_meters = y_in_cells * map_.info.resolution + map_.info.origin.position.y
            x_in_meters = x_in_cells * map_.info.resolution + map_.info.origin.position.x
            border_vertex=np.vstack([border_vertex, [x_in_meters, y_in_meters]])

        self.mapInnerBorderLength=np.abs(border_vertex[0,1]-border_vertex[1,1]) - 0.1

        self.wallLengthShort=self.mapInnerBorderLength/4
        self.wallLengthLong=2*self.wallLengthShort
        
        self.map_center=np.array([[(border_vertex[0,0]+border_vertex[1,0])/2, (border_vertex[0,1]+border_vertex[1,1])/2]])
        self.short_wall_centers=np.array([[self.map_center[0,0]+self.mapInnerBorderLength/4, self.map_center[0,1]+self.mapInnerBorderLength/4],
                                                                    [self.map_center[0,0]-self.mapInnerBorderLength/4, self.map_center[0,1]+self.mapInnerBorderLength/4], 
                                                                    [self.map_center[0,0]-self.mapInnerBorderLength/4, self.map_center[0,1]-self.mapInnerBorderLength/4], 
                                                                    [self.map_center[0,0]+self.mapInnerBorderLength/4, self.map_center[0,1]-self.mapInnerBorderLength/4]])

        self.long_wall_centers=self.map_center
        free_space_indices_new = self.update_freespace_indices_maze(map_)
        # four short walls which have the same shape (choose the first wall with theta 0 as the shape of all short walls )
        wall_shape_short=np.array([[self.short_wall_centers[0,0]-self.wallWidth/2, self.short_wall_centers[0,1]-self.wallLengthShort/2], 
                                                                    [self.short_wall_centers[0,0]+self.wallWidth/2, self.short_wall_centers[0,1]-self.wallLengthShort/2], 
                                                                    [self.short_wall_centers[0,0]+self.wallWidth/2, self.short_wall_centers[0,1]+self.wallLengthShort/2], 
                                                                    [self.short_wall_centers[0,0]-self.wallWidth/2, self.short_wall_centers[0,1]+self.wallLengthShort/2]])

        wall_shape_long=np.array([[self.long_wall_centers[0,0]-self.wallWidth/2,self.long_wall_centers[0,1]-self.wallLengthShort], 
                                                                    [self.long_wall_centers[0,0]+self.wallWidth/2,self.long_wall_centers[0,1]-self.wallLengthShort], 
                                                                    [self.long_wall_centers[0,0]+self.wallWidth/2,self.long_wall_centers[0,1]+self.wallLengthShort], 
                                                                    [self.long_wall_centers[0,0]-self.wallWidth/2,self.long_wall_centers[0,1]+self.wallLengthShort]])
        print('long shape', wall_shape_short)
        # print('long wall center3',self._wall_centers)

        return wall_shape_long, wall_shape_short, free_space_indices_new

    def update_freespace_indices_maze(self, map_: OccupancyGrid):
        """update the indices(represented in a tuple) of the freespace based on the map and the static polygons
        ostacles manuelly added 
        param map_ : original occupacy grid
        param vertlist: vertex of the polygons

        Returns:
            indices_y_x(tuple): indices of the non-occupied cells, the first element is the y-axis indices,
            the second element is the x-axis indices.
        """
        width_in_cell, height_in_cell = map_.info.width, map_.info.height
        map_2d = np.reshape(map_.data, (height_in_cell, width_in_cell))
        #height range and width range
        wall_occupancy=np.array([[1.25, 12.65, 10.6, 10.8],
                                                            [-4.45,18.35,16.3,16.5],
                                                            [-4.45, 18.35, 4.9, 5.1], 
                                                            [12.55, 12.75, -0.7, 22.1],
                                                            [1.15, 1.35, -0.7, 22.1],
                                                            [6.85, 7.05, 5.0, 16.4]])
        size=wall_occupancy.shape[0]
        for ranges in wall_occupancy:
            height_low = int(ranges[0]/map_.info.resolution)
            height_high = int(ranges[1]/map_.info.resolution)
            width_low = int(ranges[2]/map_.info.resolution)
            width_high = int(ranges[3]/map_.info.resolution)
            height_grid=height_high-height_low
            width_grid=width_high-width_low
            for i in range(height_grid):
                y =  height_low+ i
                for j in range(width_grid):
                    x= width_low + j
                    map_2d[y, x]=100
        free_space_indices_new = np.where(map_2d == 0)    
        return free_space_indices_new

    def update_maze(self):
        """
        spawn the walls of maze every episode
        return:
                pose of the walls
        """
        shortWallVertices=np.array([]).reshape(0, 4, 2)
        longWallVertices=np.array([]).reshape(0, 4, 2)
        theta_long_list=[]
        theta_short_list=[]
        long_wall_centers=np.copy(self.long_wall_centers)
        short_wall_centers=np.copy(self.short_wall_centers)
        # self.short_wall_centers
        print('shortwall centers',  self.short_wall_centers, short_wall_centers)
        for i, center in enumerate(self.long_wall_centers):
            theta=0
            delta=np.random.choice([0, np.pi/2], 1, p=[0.5, 0.5])
            theta+=delta
            theta=theta%(2*np.pi)
            theta_long_list.append(theta[0])
            if delta==0:
                vertices=np.array([[center[0] - self.wallWidth/2, center[1] - self.wallLengthShort], 
                                                        [center[0] + self.wallWidth/2, center[1] - self.wallLengthShort], 
                                                        [center[0] + self.wallWidth/2, center[1] + self.wallLengthShort], 
                                                        [center[0] - self.wallWidth/2, center[1] + self.wallLengthShort]])
                vertices=vertices.reshape(1, 4, 2)
                long_wall_centers[i, 1] = 0.0
                long_wall_centers[i, 0] = 0.0
                longWallVertices=np.concatenate((longWallVertices, vertices), axis=0)
            else:
                vertices=np.array([[center[0] - self.wallLengthShort, center[1] - self.wallWidth/2], 
                                                        [center[0] - self.wallLengthShort, center[1] + self.wallWidth/2], 
                                                        [center[0] + self.wallLengthShort,center[1] + self.wallWidth/2], 
                                                        [center[0] + self.wallLengthShort,center[1] - self.wallWidth/2]])
                vertices=vertices.reshape(1, 4, 2)
                long_wall_centers[i, 1] = -3.75
                long_wall_centers[i, 0] = 17.65
                longWallVertices=np.concatenate((longWallVertices, vertices), axis=0)

        for i, center in enumerate(self.short_wall_centers):
            theta=0 
            delta=np.random.choice([0, np.pi/2], 1, p=[1, 0])
            theta+=delta
            theta=theta%(2*np.pi)
            theta_short_list.append(theta[0])
            sign=np.random.choice([-1, 1], 1, p=[0.5, 0.5])
            # vertices of the walls
            if delta==0:
                short_wall_centers[i, 1]  =short_wall_centers[i, 1]+ sign*self.mapInnerBorderLength/8
                vertices= np.array([[center[0]-self.wallWidth/2, center[1]-self.wallLengthShort/2], 
                                                        [center[0]+self.wallWidth/2, center[1]-self.wallLengthShort/2], 
                                                        [center[0]+self.wallWidth/2, center[1]+self.wallLengthShort/2], 
                                                        [center[0]-self.wallWidth/2, center[1]+self.wallLengthShort/2]])
                vertices=vertices.reshape(1, 4, 2)
                shortWallVertices= np.concatenate((shortWallVertices, vertices), axis=0)
            else:
                short_wall_centers[i, 0]   = short_wall_centers[i, 1] + sign*self.mapInnerBorderLength/8
                vertices= np.array([[center[0]-self.wallLengthShort/2, center[1]-self.wallWidth/2], 
                                                        [center[0]-self.wallLengthShort/2, center[1]+self.wallWidth/2], 
                                                        [center[0]+self.wallLengthShort/2, center[1]+self.wallWidth/2], 
                                                        [center[0]+self.wallLengthShort/2, center[1]-self.wallWidth/2]])
                vertices=vertices.reshape(1, 4, 2)
                shortWallVertices= np.concatenate((shortWallVertices, vertices), axis=0)
            # translations and rotations of the walls
            if i==0:
                if delta==0:
                    short_wall_centers[i, 1] = sign*self.mapInnerBorderLength/8
                    short_wall_centers[i, 0] = 0.0
                else:
                    short_wall_centers[i, 1] = -3.75
                    short_wall_centers[i, 0]  = 29.05 + sign*self.mapInnerBorderLength/8
            elif i==1:
                if delta==0:
                    short_wall_centers[i, 1] = sign*self.mapInnerBorderLength/8
                    short_wall_centers[i, 0] = -11.4
                else:
                    short_wall_centers[i, 1] = -3.75
                    short_wall_centers[i, 0]  = -11.4+29.05 + sign*self.mapInnerBorderLength/8
            elif i==2:
                if delta==0:
                    short_wall_centers[i, 1] = -11.4+sign*self.mapInnerBorderLength/8
                    short_wall_centers[i, 0] = -11.4
                else:
                    short_wall_centers[i, 1] = -11.4 - 3.75
                    short_wall_centers[i, 0]  = -11.4+29.05 + sign*self.mapInnerBorderLength/8
            elif i==3:
                if delta==0:
                    short_wall_centers[i, 1] = -11.4 + sign*self.mapInnerBorderLength/8
                    short_wall_centers[i, 0] = 0.0
                else:
                    short_wall_centers[i, 1] = - 11.4 - 3.75
                    short_wall_centers[i, 0]  = 29.05 + sign*self.mapInnerBorderLength/8

        return short_wall_centers, long_wall_centers, theta_short_list, theta_long_list, shortWallVertices, longWallVertices


