import numpy as np
import matplotlib.pyplot as plt

array = np.load("/home/daniel/catkin_ws/src/arena-rosnav/simulator_setup/scripts/behavior_modeling/potential_field.npy")
norms = np.linalg.norm(array, axis=2)
norms = np.expand_dims(norms, 2)
norms = np.repeat(norms, 2, axis=2)
# array = array / norms
# array *= 100
# print(array[50,50])

# u = array[:,:,0]
# v = array[:,:,1]

u = array[:100:1,:100:1,0]
v = array[:100:1,:100:1,1]

plt.quiver(u, v)
plt.show()
