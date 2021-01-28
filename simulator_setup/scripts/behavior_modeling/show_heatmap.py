import numpy as np
import matplotlib.pyplot as plt
from BehaviorModel import BehaviorModel

# read in logs

logs_folder = ""
positions_path = logs_folder + "positions.txt"
velocities_path = logs_folder + "velocities.txt"

positions_list = []
with open(positions_path, "r") as f:
    for line in f:
        x_str, y_str = line.split(" ")
        x = float(x_str)
        y = float(y_str)
        positions_list.append([x, y])
positions = np.array(positions_list)
print("positions: ", positions.shape)

velocities_list = []
with open(velocities_path, "r") as f:
    for line in f:
        vel = float(line.strip())
        velocities_list.append(vel)
velocities = np.array(velocities_list)
print("velocities: ", velocities.shape)

# get heatmap

model = BehaviorModel(positions, velocities)

# plot

# imgplot = plt.imshow(model.positions_moving)
imgplot = plt.imshow(model.positions_standing)

plt.show()
