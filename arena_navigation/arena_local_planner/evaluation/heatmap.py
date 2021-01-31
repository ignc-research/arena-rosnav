import numpy as np
import seaborn as sb
import matplotlib.pyplot as plt
import random

data = np.random.rand(25, 25)
data = np.zeros((20, 20))
print(data.shape)



rows = data.shape[0]
cols = data.shape[1]

for x in range(0, cols):
    for y in range(0, rows):
        val = random.randint(x-y,20)
        if val<0:
            val = 0
        data[x,y] = val



heat_map = sb.heatmap(data,  cmap="YlGnBu")
heat_map.invert_yaxis()
plt.show()