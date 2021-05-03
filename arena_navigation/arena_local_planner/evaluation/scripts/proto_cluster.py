import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# X = np.array([[5,3],
#     [10,15],
#     [15,12],
#     [24,10],
#     [30,30],
#     [85,70],
#     [71,80],
#     [60,78],
#     [70,55],
#     [80,91],])



# cluster = AgglomerativeClustering(n_clusters=2, affinity='euclidean', linkage='ward')
# cluster.fit_predict(X)

# print(cluster.labels_)
# plt.scatter(X[:,0],X[:,1], c=cluster.labels_, cmap='rainbow')
# plt.show()

# %%
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
# %%
dates = ['2016-1-1', '2016-1-2', '2016-1-3']
cols = pd.MultiIndex.from_product([dates, ['High', 'Low']])
cols
# pd.DataFrame(data=cols)
# %%
bags = {}
pose_x = np.asarray([1,2,3,4]).T

pose_y = np.asarray([2,2,3,4]).T
t = np.asarray([3,2,3,4]).T
col_xy = np.asarray([4,2,3,4]).T
subgoal_x = np.asarray([5,2,3,4]).T
subgoal_y = np.asarray([6,2,3,4]).T
wpg_x = np.asarray([7,2,3,4]).T
wpg_y = np.asarray([8,2,3,4]).T
bags["run_1"] = [pose_x, pose_y, t, col_xy, subgoal_x, subgoal_y, wpg_x, wpg_y]
bags["run_2"] = [pose_x, pose_y, t, col_xy, subgoal_x, subgoal_y, wpg_x, wpg_y]
# %%
df = pd.DataFrame(data=bags)
df2 = df.to_dict()
df.to_csv("test.csv",index=False)
# %%
df
# %%
df2
# %%
runs = pd.read_excel('runs_ex.xlsx',engine='openpyxl') 
type(runs)
runs.to_excel("output.xlsx") 
# %%
df2["run_2"]
# %%
bags["run_2"]
# %%
import json

data = {}
data['run']       = []
data['time']      = []
data['path']      = []
data['velocity']  = []
data['collision'] = []

data['run'].append(0)
data['time'].append(1)
data['path'].append(2)
data['velocity'].append(3)
data['collision'].append(4)



with open('data.json', 'w') as outfile:
    json.dump(data, outfile)
# %%
