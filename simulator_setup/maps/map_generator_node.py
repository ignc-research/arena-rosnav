from map_generator import *

if __name__ == "__main__":
    if np.random.random()>=0.5:
        create_indoor_map(
            height = 101,
            width = 101,
            corridor_radius = 3,
            iterations = 100
        )
    else:
        create_outdoor_map(
            height = 101,
            width = 101,
            obstacle_number = 20,
            obstacle_extra_radius = 2 # extra radius, obstacle_extra_radius=0 equals 1 pixel obstacles
        )