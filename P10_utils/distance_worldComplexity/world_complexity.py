import math
import cv2
import sys
import yaml
import os
# import rospkg
import numpy as np
from argparse import ArgumentParser
from collections import Counter
from scipy import spatial
from pathlib import Path
from collections import namedtuple
from distance_between import Distance
import progressbar
from tqdm import tqdm

# TODO s
# - die angle liste soll man trennen, weil gerade in einer liste gespeichert werden und ich weiß nicht, welche in welche

# see comments at the end of the document

ROBOT_RADIUS = 0.3  # [m]
MIN_INTENDED_WALKWAY_WITH = 0.2  # [m]
# [m]: the obs must be at least 20 cm to avoid detecting obstacles
# that are not actually there due to miscoloured pixel
MIN_OBS_SIZE = 0.2


def write2File(filepath, mapdata):
    file = open(filepath, 'w')
    file.truncate(0)
    file.write('World, EntropyRatio, MapSize, OccupancyRatio, NumObs_Cv2, AngleInfo_mean, distance_norm, distance_var, distance_avg\n')

    for element in mapdata:
        file.write(element)
        file.write('\n')
    file.close()


class Complexity:
    def __init__(self):
        self.density_gird = []

    def extract_data(self, img_path: str, yaml_path: str):

        # reading in the image and converting to 0 = occupied, 1 = not occupied
        if img_path[-3:] == "pgm" or img_path[-3:] == "jpg" or img_path[-3:] == "png":
            img_origin = cv2.imread(img_path)
            img = cv2.cvtColor(img_origin, cv2.COLOR_BGR2GRAY)
            # convert image pixels to binary pixels 0 or 255
            th, dst = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY)

        # for infos on parameters: http://wiki.ros.org/map_server
        with open(yaml_path, "r") as stream:
            map_info = yaml.safe_load(stream)

        # this only selects interior area
        t = np.where(img == 0.0)
        x_low_lim = min(t[0])
        x_high_lim = max(t[0])
        y_low_lim = min(t[1])
        y_high_lim = max(t[1])

        test = img[x_low_lim:x_high_lim, y_low_lim:y_high_lim]
        # cv2.imshow('test',test)
        # cv2.waitKey(0)
        return img_origin, img, map_info

    def determine_map_size(self, img: list, map_info: dict):
        """Determining the image size by using resolution in meters
        """
        return map_info["resolution"] * img.shape[0] * img.shape[1]

    def occupancy_ratio(self, img: list):
        """Proportion of the occupied area
        """
        # TODO to find the actual occupancy only interior occupied pixels should be taken into account
        # idea get the pos on the sides (a,b,c,d) where the value is first 0,
        # use: https://stackoverflow.com/questions/9553638/find-the-index-of-an-item-in-a-list-of-lists
        free_space = np.count_nonzero(img)
        return 1 - free_space / (img.shape[0] * img.shape[1])

    def occupancy_distribution(self, img: list):
        # Idea: https://jamboard.google.com/d/1ImC7CSPc6Z3Dkxh5I1wX_kkTjEd6GFWmMywHR3LD_XE/viewer?f=0
        raise NotImplementedError

    def entropy(self, img_gray):
        """ Measures the amount of disorder of in the image (between the obstacles).
            Proposed here: Performance Measure For The Evaluation of Mobile Robot Autonomy
        """
        new_img = self.gray_bg_square(img_gray)
        # cv2.imshow('map_gray', img_gray)
        # print(img_gray)
        # cv2.imshow('new_img', new_img)
        # print(new_img)
        # cv2.waitKey()
        # print()
        th, dst = cv2.threshold(new_img, 0, 255, cv2.THRESH_BINARY)
        windows = self.sliding_window(dst, 2, (2, 2))
        windowList = []
        for window in windows:
            windowList.append(window)
            featureVector = self.extractFeatures(window)
        pList = []
        count = Counter(featureVector)
        p_zero = count[0.0] / len(featureVector)
        p_one = count[1] / len(featureVector)
        p_two = count[0.25] / len(featureVector)
        p_five = count[0.5] / len(featureVector)
        p_seven = count[0.75] / len(featureVector)
        pList.append(p_zero)
        pList.append(p_one)
        pList.append(p_two)
        pList.append(p_five)
        pList.append(p_seven)
        entropy = 0
        for pDensity in pList:
            if pDensity != 0:
                entropy += (pDensity) * np.log(pDensity)
        entropy = (-1) * entropy
        maxEntropy = np.log2(5)
        #  print('calculated entropy:', entropy)
        # print('Max. Entropy:', maxEntropy)
        return float(entropy), float(maxEntropy)

    def sliding_window(self, image, stepSize, windowSize):
        for y in range(0, image.shape[0], stepSize):
            for x in range(0, image.shape[1], stepSize):
                yield (x, y, image[y : y + windowSize[1], x : x + windowSize[0]])

    def extractFeatures(self, window):

        freq_obstacles = window[2] == 0
        total = freq_obstacles.sum()
        density = total * 1 / 4
        self.density_gird.append(density)

        return self.density_gird

    def gray_bg_square(self, img):
        # "return a white-background-color image having the img in exact center"

        old_image_height, old_image_width = img.shape

        # create new image of desired size and color (blue) for padding
        image = cv2.copyMakeBorder(
            img,
            500 - int(img.shape[0] / 2),
            500 - int(img.shape[0] / 2),
            500 - int(img.shape[1] / 2),
            500 - int(img.shape[1] / 2),
            cv2.BORDER_REPLICATE,
        )
        # new_image_width = 1000
        # new_image_height = 1000
        # color = (205,205,205)
        # result = np.full((new_image_height,new_image_width, 3), color, dtype=np.uint8)
        #
        # # compute center offset
        # x_center = (new_image_width - old_image_width) // 2
        # y_center = (new_image_height - old_image_height) // 2
        #
        # # copy img image into center of result image
        # result[y_center:y_center+old_image_height, x_center:x_center+old_image_width] = img
        # view result
        return image

    def check_surrounding(self, img, i, j, obs_coordinates):
        """Determining all pixels that are occupied by this obstacle
        args:
            img: the floor plan
            i,j: start coordinate of the obstacle
            obs_coordinates: list of coordinates occupied by this obstacle

        return:
            img: floor plan without the obstacle on it
            obs_coordinates: tuple of all pixel-indices occupied by the obstacle
        """

        # marking the pixel and setting it to not occupied to avoid double assignment
        obs_coordinates.append((i, j))
        img[i, j] = 205

        # for every point we check in a star pattern, whether the surrounding cells are occupied also
        if i + 1 < img.shape[0]:
            if j + 1 < img.shape[1]:

                if img[i - 1][j] == 0 and i >= 1:
                    self.check_surrounding(img, i - 1, j, obs_coordinates)
                if img[i + 1][j] == 0 and i + 1 <= img.shape[0]:
                    self.check_surrounding(img, i + 1, j, obs_coordinates)
                if img[i][j - 1] == 0 and j >= 1:
                    self.check_surrounding(img, i, j - 1, obs_coordinates)
                if img[i][j + 1] == 0 and i + 1 <= img.shape[1]:
                    self.check_surrounding(img, i, j + 1, obs_coordinates)

        return img, obs_coordinates

    def number_of_static_obs(self, img: list):
        """Determining the obstacle in the image incl. their respective pixels
        args:
            img: floorplan to evaluate
        """
        # to allow more recursive function calls
        sys.setrecursionlimit(20000)

        global obs_list
        obs_list = {}
        obs_num = 0

        # number of pixels an obstacle must have at least (default = 2)
        min_pix_size = MIN_OBS_SIZE / map_info["resolution"]

        # going through every pixel and checking if its occupied
        for pixel_y in range(img.shape[1]):
            for pixel_x in range(img.shape[0]):
                if img[pixel_x, pixel_y] == 0:
                    obs_coordinates = []
                    img, temp = self.check_surrounding(
                        img, pixel_x, pixel_y, obs_coordinates
                    )
                    if len(temp) >= min_pix_size:
                        obs_list[obs_num] = temp
                        obs_num += 1

        #  print('obs_list', len(obs_list))
        return len(obs_list)

    def distance_between_obs(self):
        """Finds distance to all other obstacles
        """

        def do_kdtree(combined_x_y_arrays, points):
            mytree = spatial.cKDTree(combined_x_y_arrays)
            dist, _ = mytree.query(points)
            return dist

        # the walkway should be at least this whide to be considert has walkway
        min_walkway_size = MIN_INTENDED_WALKWAY_WITH / map_info["resolution"]

        min_obs_dist = 10000  # large number
        for key, coordinates in obs_list.items():

            distances = []
            for key_other, coordinates_other in obs_list.items():
                if key_other == key:
                    continue
                # this checks the distance between the pixels of the obstacles, and only selects the min distance between the pixels
                dist_between_pixel_arrays = do_kdtree(
                    np.array(coordinates_other), np.array(coordinates)
                )
                filtered_pix = dist_between_pixel_arrays[
                    dist_between_pixel_arrays > min_walkway_size
                ]
                if len(filtered_pix) > 0:
                    min_dist_between_pixel_arrays = min(filtered_pix)
                if min_obs_dist > min_dist_between_pixel_arrays:
                    min_obs_dist = min_dist_between_pixel_arrays

        #   print(min_obs_dist)

        return float(min_obs_dist * map_info["resolution"])

    def no_obstacles(self, path):
        """ finds all obstacles and determines there center
        """
        areaList = []
        xcoordinate_center = []
        ycoordinate_center = []
        cnt, image = Distance().find_Contours(path)

        # print("No of circles: ", len(cnt))
        for c in cnt:
            if cv2.contourArea(c) > 1:
                area = int(cv2.contourArea(c))
                areaList.append(area)
                length = int(cv2.arcLength(c, True))
                M = cv2.moments(c)
                xcoord = int(M["m10"] / M["m00"])
                ycoord = int(M["m01"] / M["m00"])
                xcoordinate_center.append(xcoord)
                ycoordinate_center.append(ycoord)
                coordList = [area, length, xcoord, ycoord]
        self.world_angles(xcoordinate_center, ycoordinate_center)

        cv2.waitKey(500)
        return xcoordinate_center, ycoordinate_center, len(cnt)

    def pixelVal(self, pix, r1, s1, r2, s2):

        if 0 <= pix and pix <= r1:
            return (s1 / r1) * pix
        elif r1 < pix and pix <= r2:
            return ((s2 - s1) / (r2 - r1)) * (pix - r1) + s1
        else:
            return ((255 - s2) / (255 - r2)) * (pix - r2) + s2

    def world_angles(self, xCoord: list, yCoord: list):
        """ calculates the angels between the obstacles in each window
        args:
            xCoord: x-coordinate of the center of each obs
            yCoord: y-coordinate of the center of each obs
        """

        xIndices = []
        xIndices_interval = []
        intervalPointList = []
        angle = []
        subAngleList = []

        xInterval = [
            min(xCoord) - 10,
            min(xCoord) + 10,
        ]  # finds the smallest x_coordination and forms an interval with +-10
        # print('xInterval', xInterval)
        # find the index of this point to find the corresponding y_coordinate
        for (index, item) in enumerate(xCoord):
            if item == min(xCoord):
                xIndices.append(index)

        xMax = max(xCoord)
        yMin = min(yCoord)
        yMax = max(yCoord)

        # because we want the point with smallest x and y, we should check if y is also the smallest
        if yCoord[xIndices[0]] - 10 > yMin:
            yInterval = [yMin - 1, yCoord[xIndices[0]] + 10]
        else:
            yInterval = [yCoord[xIndices[0]] - 10, yCoord[xIndices[0]] + 10]

        orgYInterval = yInterval
        listLength = len(xCoord)
        counter = 0

        # loop interval in y direction
        while yInterval[1] <= yMax + 10 and xInterval[1] <= xMax + 10:
            for (index, item) in enumerate(
                (xCoord)
            ):  # we find all points that are in this interval
                counter += 1

                # find x and indices of x in interval
                if item <= xInterval[1] and item >= xInterval[0]:
                    xIndices_interval.append(index)
                    # find y with indices of x
                    if yCoord[index] <= yInterval[1] and yCoord[index] >= yInterval[0]:
                        # find coordination of point in interval
                        intervalPointList.append([item, yCoord[index]])
                        point = [item, yCoord[index]]
                        xCenter = (xInterval[1] - xInterval[0]) / 2
                        yCenter = (yInterval[1] - yInterval[0]) / 2
                        # calculate angle of all points in interval to the first x,y of interval
                        angle.append(
                            self.get_angle(
                                [xInterval[0] + xCenter, yInterval[0] + yCenter], point
                            )
                        )

                if (
                    listLength == counter
                ):  # when the y interval is over, window should go in x direction and then again in y direction, to find the new points
                    yInterval = [yInterval[1], yInterval[1] + 10]
                    sortedAngle = sorted(angle)
                    for index in range(len(sortedAngle)):
                        if index != 0:
                            sub = sortedAngle[index] - sortedAngle[index - 1]
                            subAngleList.append(sub)
                    sumAngles = sum(subAngleList)
                    lastAngle = 360 - sumAngles
                    subAngleList.append(lastAngle)

                    global list_of_all_angles

                    list_of_all_angles.append(subAngleList)
                    subAngleList = []
                    angle = []
                    counter = 0

            if not (yInterval[1] <= yMax + 10):
                xInterval = [xInterval[1], xInterval[1] + 10]
                yInterval = orgYInterval

    Point = namedtuple("Point", ["x", "y"])

    def get_angle(self, p1: Point, p2: Point) -> float:
        """Get the angle of this line with the horizontal axis."""

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        theta = math.atan2(dy, dx)
        angle = math.degrees(theta)  # angle is in (-180, 180]
        if angle < 0:
            angle = 360 + angle

        return angle

    def processing_angle_information(self):
        """Evaluates the angle information of the obstacles, by taking in a list of all angles as input
        """
        global list_of_all_angles
        angle_data = {}
        nr_windows = len(list_of_all_angles)
        list_of_all_angles = [
            item for sublist in list_of_all_angles for item in sublist
        ]  # flatten the list
        angle_data["mean"] = float(np.mean(list_of_all_angles))
        # angle_data['variance'] = float(np.var(list_of_all_angles))
        # if 0 not in [angle_data['mean'], angle_data['variance']]:
        #  angle_data['adjusted_mean'] = float(
        #  angle_data['mean'] / angle_data['variance'])

        return angle_data

    def save_information(self, data: dict, dest: str):
        """To save the evaluated metrics
        args:
            data: data of the evaluated metrics
            dest: path were the data should be saved
        """
        os.chdir(dest)
        with open(f"complexity.yaml", "w+") as outfile:
            yaml.dump(data, outfile, default_flow_style=False)


if __name__ == "__main__":
    print(
        "--------------------------------------------------------------------------------"
    )
    print("calculation starts: ")

    #    dir = rospkg.RosPack().get_path('arena-tools')
    # reading in user data
    parser = ArgumentParser()
    parser.add_argument(
        "--image_path",
        action="store",
        dest="image_path",
        default=f"/home/elias/catkin_ws/src/arena-rosnav-3D/simulator_setup/maps/ignc/map.pgm",
        help="path to the floor plan of your world. Usually in .pgm format",
        required=False,
    )
    parser.add_argument(
        "--yaml_path",
        action="store",
        dest="yaml_path",
        default=f"/home/elias/catkin_ws/src/arena-rosnav-3D/simulator_setup/maps/ignc/map.yaml",
        help="path to the .yaml description file of your floor plan",
        required=False,
    )
    parser.add_argument(
        "--dest_path",
        action="store",
        dest="dest_path",
        help="location to store the complexity data about your map",
        required=False,
    )
    parser.add_argument(
        "--folders_path",
        action="store",
        dest="folders_path",
        help="path of maps folders (all maps and yaml fiels are in separate folders)",
        required=False,
    )

    args = parser.parse_args()
    converted_dict = vars(args)
    file_name = Path(converted_dict["image_path"]).stem

    # extract data
    if args.folders_path:

        dirs = [x[0] for x in os.walk(args.folders_path)][1:]
        mapsdata = []

        for dir in tqdm(dirs):

            folders = [x[2] for x in os.walk(dir)]
            png = [s for s in folders[0] if ".png" in s]
            yml = [s for s in folders[0] if "map.yaml" in s]
            image_path = dir + "/{}".format(png[0])
            yaml_path = dir + "/{}".format(yml[0])
            dest_path = dir

            img_origin, img, map_info = Complexity().extract_data(image_path, yaml_path)

            data = {}
            list_of_all_angles = []
            _, _, num = Complexity().no_obstacles(image_path)

            # calculating metrics
            data["Entropy"], data["MaxEntropy"] = Complexity().entropy(img)
            data["MapSize"] = Complexity().determine_map_size(img, map_info)
            data["OccupancyRatio"] = Complexity().occupancy_ratio(img)
            # data["NumObs_pixels"] = Complexity().number_of_static_obs(img)
            data["NumObs_Cv2"] = num
            # data["MinObsDis"] = Complexity().distance_between_obs()
            data["AngleInfo"] = Complexity().processing_angle_information()
            (
                data["distance(normalized)"],
                data["distance.variance"],
                _,
                data["distance.avg"],
            ) = Distance().image_feat(image_path, 0.3)

            # dump results
            Complexity().save_information(data, dest_path)

            # save mapdata in csv file
            f_name = os.path.basename(image_path).split('.')[0]

            csv_str = str(f_name) + ',' + str(data["Entropy"] / data["MaxEntropy"]) + ',' + str(data['MapSize']) + ',' \
                      + str(data["OccupancyRatio"]) + ',' + str(num) + ',' + str(data["AngleInfo"]["mean"]) + ',' \
                      + str(data['distance(normalized)']) + ',' + str(data['distance.variance']) + ',' + str(data['distance.avg'])
            mapsdata.append(csv_str)
        #print(mapsdata)

        # write to file
        csvPath = args.folders_path + '/map_worldcomplexity_results.csv'
        write2File(csvPath, mapsdata)

    else:

        img_origin, img, map_info = Complexity().extract_data(
            args.image_path, args.yaml_path
        )

        data = {}
        list_of_all_angles = []
        _, _, num = Complexity().no_obstacles(args.image_path)

        # calculating metrics
        data["Entropy"], data["MaxEntropy"] = Complexity().entropy(img)
        data["MapSize"] = Complexity().determine_map_size(img, map_info)
        data["OccupancyRatio"] = Complexity().occupancy_ratio(img)
        # data["NumObs_pixels"] = Complexity().number_of_static_obs(img)
        data["NumObs_Cv2"] = num
        # data["MinObsDis"] = Complexity().distance_between_obs()
        data["AngleInfo"] = Complexity().processing_angle_information()
        (
            data["distance(normalized)"],
            data["distance.variance"],
            _,
            data["distance.avg"],
        ) = Distance().image_feat(args.image_path, 0.3)

        # dump results
        Complexity().save_information(data, args.dest_path)

        bar = progressbar.ProgressBar(
            maxval=20,
            widgets=[progressbar.Bar("=", "[", "]"), " ", progressbar.Percentage()],
        )

    # print(data)

# NOTE: for our complexity measure we make some assumptions
# 1. We ignore the occupancy threshold. Every pixel > 0 is considert to be fully populated even though this is not entirely accurate since might also only partially be populated (we therefore slightly overestimate populacy.)
# 2. We assume the map shows only areas relevant for robot navigation (interior areas). For example having a large exterior area on your map, could lead do biased occupancy-ratio measurements
