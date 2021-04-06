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
        self.num_rays = 8
        # obstacles beyond this radius are not considered
        self.effective_force_radius = 5.0
        # map height and width
        self.height = 0
        self.width = 0

    def obstacle_force(self, distance):
        if distance <= 0:
            return 0.0
        return 1 / distance

    def get_quadrant(self, point):
        # point: point in 2D space
        # return: corresponding quadrant (1, 2, 3, or 4)
        x = point[0]
        y = point[1]
        if x == 0 and y == 0:
            return 0
        elif x > 0 and y > 0:
            return 1
        elif x < 0 and y > 0:
            return 2
        elif x < 0 and y < 0:
            return 3
        elif x > 0 and y < 0:
            return 4
        else:
            return -1

    def mirror_to_quadrant(self, point, quadrant_to):
        # point: point in 2D space
        # quadrant_to: integer, quadrant to mirror to
        # return: mirrored point
        assert quadrant_to in [1, 2, 3, 4]

        quadrant_from = self.get_quadrant(point)

        if quadrant_from == 0:
            return point
        elif quadrant_from == quadrant_to:
            return point
        else:
            # mirror to next quadrant
            mirrored_point = np.array()
            if quadrant_from == 1:
                mirrored_point = np.array([point[0] * -1, point[1]])
            elif quadrant_from == 2:
                mirrored_point = np.array([point[0], point[1] * -1])
            elif quadrant_from == 3:
                mirrored_point = np.array([point[0] * -1, point[1]])
            elif quadrant_from == 4:
                mirrored_point = np.array([point[0], point[1] * -1])

            return self.mirror_to_quadrant(mirrored_point, quadrant_to)


    # TODO just implement for quadrant 1 and rotate back and forth
    def get_covered_cells(self, start, end):
        '''
        Find map cells covered by line.
        Returns an Nx2 array containing (x,y) points.
        '''
        # y0 = int(start[0])
        # x0 = int(start[1])
        # y1 = int(end[0])
        # x1 = int(end[1])

        y0 = 0
        x0 = 0
        y1 = int(end[0] - start[0])
        x1 = int(end[1] - start[1])

        deltax = x1 - x0
        if deltax == 0:
            # no change in x value
            # get vertical line going up from minimum y to maximum y
            res = np.array([np.array([x0, y]) for y in np.arange(min(y0, y1), max(y0, y1) + 1)])
            # flip if end point is lower than start point
            return res if y0 <= y1 else np.flip(res, axis=0)

        m = float(y1 - y0) / float(x1 - x0)
        line_func = lambda x: m * x + y0
        cells = []

        x_step = np.sign(x1 - x0)
        xs = np.arange(x0, x1 + x_step, step=x_step)
        for x in xs:
            # get min and max y values in line segment by calculating leftmost and rightmost value
            y_left = line_func(x - 0.5)
            y_right = line_func(x + 0.5)

            # generate all cells between (and including) (x, y_left) and (x, y_right)
            y_left_rounded = int(np.rint(y_left))
            y_right_rounded = int(np.rint(y_right))
            # y_step = int(np.sign(m)) if x_step > 0 else -1 * int(np.sign(m))
            y_step = int(np.sign(m))
            
            ## slope is 0; just take current cell
            if y_step == 0:
                y = int(line_func(x))
                cells.append(np.array([x, y]))
                continue

            ys = np.arange(y_left_rounded, y_right_rounded + y_step, step=y_step)
            for y in ys:
                # cells.append(np.array([x, y]))
                # TODO is this correct?
                moved_y = y + start[0]
                moved_x = x + start[1]
                if 0 <= moved_y < self.height and 0 <= moved_x < self.width:
                    cells.append(np.array([moved_y, moved_x]))

        return np.array(cells)


    # def BLA(self, start, end):
    #     '''
    #     Use Bresenhams line algorithm to find cells covered by line.
    #     Returns an Nx2 array containing (x,y) points.
    #     '''
    #     x0 = start[0]
    #     y0 = start[1]
    #     x1 = end[0]
    #     y1 = end[1]
    #     deltax = x1 - x0
    #     if deltax == 0:
    #         # no change in x value
    #         # get vertical line going up from minimum y to maximum y
    #         res = np.array([np.array([x0, y]) for y in np.arange(min(y0, y1), max(y0, y1) + 1)])
    #         # flip if end point is lower than start point
    #         return res if y0 <= y1 else np.flip(res, axis=0)
    #     deltay = y1 - y0
    #     deltaerr = abs(deltay / deltax)
    #     error = 0.0
    #     y = y0
    #     pixels = []
    #     xs = []
    #     if x0 < x1:
    #         xs = np.arange(x0, x1+1)
    #     else:
    #         xs = np.flip(np.arange(x1, x0+1))
    #     for x in xs:
    #         pixels.append(np.array([x, y]))
    #         error += deltaerr
    #         if error >= 0.5:
    #             y += np.sign(deltay)
    #             error -= 1.0
    #     return np.array(pixels)


    def map_callback(self, map):
        self.height = map.info.height
        self.width = map.info.width
        print(self.height, self.width)
        map_array = np.array(map.data).reshape(map.info.height, map.info.width)
        if self.potential_field == None:
            # one 2D force vector for every map cell
            self.potential_field = np.zeros((map.info.height, map.info.width, 2))
            
            circumference = 2 * np.pi * self.effective_force_radius
            delta_angle = circumference / float(self.num_rays)  # angle in radians
            for h in range(map.info.height):
                for w in range(map.info.width):
                    force = np.zeros(2)
                    print(h, w)

                    for k in range(self.num_rays):
                        current_pos = np.array([h, w])

                        # project ray at angle and get end position
                        angle = k * delta_angle
                        y = np.sin(angle) * self.effective_force_radius + h
                        x = np.cos(angle) * self.effective_force_radius + w
                        ray_end_pos = np.array([y, x], dtype=int)

                        # get cells covered by ray
                        # TODO gives back empty array when called with (0, 0) and (-1, -4)
                        cell_coords = self.get_covered_cells(start = current_pos, end = ray_end_pos)
                        # print(current_pos)
                        # print(ray_end_pos)
                        # print(cell_coords.shape)
                        # # get occupancy values for the cells
                        # print("---------")
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
            
            np.save("/home/daniel/Desktop/potential_field.npy", self.potential_field)
            print("saved potential field to file")
        pass
        
    def listen(self):
        rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size = 1)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    listener = Listener()
    listener.listen()

# if __name__ == '__main__':
#     listener = Listener()
#     listener.height = 521
#     listener.width = 666
#     start = [0, 662]
#     end = [2, 666]
#     cells = listener.get_covered_cells(start, end)
#     cells = np.array(cells)
#     print(cells)
