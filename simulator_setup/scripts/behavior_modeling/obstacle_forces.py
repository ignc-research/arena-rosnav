#!/usr/bin/env python
import numpy as np
import rospy
import sys
import time
from nav_msgs.msg import OccupancyGrid

class Listener:
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        self.potential_field = None
        # number of equally spaced rays to scan for obstacles
        self.num_rays = 36
        # obstacles beyond this radius are not considered
        self.effective_force_radius = 5.0

    def obstacle_force(self, distance):
        assert distance > 0
        return 1 / distance

    def BLA(self, start, end):
        '''
        Use Bresenhams line algorithm to find cells covered by line.
        Returns an Nx2 array containing (x,y) points.
        '''
        x0 = start[0]
        y0 = start[1]
        x1 = end[0]
        y1 = end[1]
        deltax = x1 - x0
        if deltax == 0:
            res = np.array([np.array([x0, y]) for y in np.arange(min(y0, y1), max(y0, y1) + 1)])
            return res if y0 <= y1 else np.flip(res, axis=0)
        deltay = y1 - y0
        deltaerr = abs(deltay / deltax)
        error = 0.0
        y = y0
        pixels = []
        for x in np.arange(x0, x1+1):
            pixels.append(np.array([x, y]))
            error += deltaerr
            if error >= 0.5:
                y += np.sign(deltay)
                error -= 1.0
        return np.array(pixels)


    def map_callback(self, map):
        map_array = np.array(map.data).reshape(map.info.height, map.info.width)
        if self.potential_field == None:
            # one 2D force vector for every map cell
            self.potential_field = np.zeros((map.info.height, map.info.width, 2))
            
            circumference = 2 * np.pi * self.effective_force_radius
            delta_angle = circumference / float(self.num_rays)  # angle in radians
            for h in range(map.info.height):
                for w in range(map.info.width):
                    force = np.zeros(2)

                    for k in range(self.num_rays):
                        current_pos = np.array([h, w])

                        # project ray at angle and get end position
                        angle = k * delta_angle
                        y = np.sin(angle) * self.effective_force_radius + h
                        x = np.cos(angle) * self.effective_force_radius + w
                        ray_end_pos = np.array([y, x], dtype=int)

                        # get cells covered by ray
                        # TODO gives back empty array when called with (0, 0) and (-1, -4)
                        cell_coords = self.BLA(start = current_pos, end = ray_end_pos)
                        print(current_pos)
                        print(ray_end_pos)
                        print(cell_coords.shape)
                        print("---------")
                        # get occupancy values for the cells
                        occupancies = map_array[cell_coords[:,0], cell_coords[:,1]]
                        # get nearest obstacle
                        first_occupancy_index = np.argmax(occupancies > 0)
                        if first_occupancy_index == 0:
                            continue
                        obstacle_pos = cell_coords[first_occupancy_index]
                        v = current_pos - obstacle_pos
                        v_len = np.linalg.norm(v)
                        direction = v / v_len
                        force_magnitude = self.obstacle_force(v_len)
                        force += direction * force_magnitude

                    self.potential_field[h, w] = force
        pass
        
    def listen(self):
        rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size = 1)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    listener = Listener()
    listener.listen()