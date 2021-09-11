import numpy as np 
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import csv
import os
import pandas as pd
import random 
import matplotlib.path as mpath
import matplotlib.patches as mpatches
from scipy.interpolate import make_interp_spline, BSpline


# pts = [(0,0), (0,1), (3,1), (3,0)] # Corners of rectangle of height 1, length 3
# apts = np.array(pts) # Make it a numpy array
# lengths = np.sqrt(np.sum(np.diff(apts, axis=0)**2, axis=1)) # Length between corners
# total_length = np.sum(lengths)


data = pd.read_csv('data/evaluation_360.csv')

done_reason = np.array(data['done_reason'])
episode = np.array(data['episode'])
task_flag = np.array(data['task_flag'])
vip_rho = np.array(data['vip_rho'])
robot_orientation = np.array(data['robot_orientation'])
vip_orientation = np.array(data['vip_orientation'])
robot_velocity = np.array(data['robot_velocity'])
vip_velocity = np.array(data['vip_velocity'])
robot_pos_x = np.array(data['robot_pos_x'])
robot_pos_y = np.array(data['robot_pos_y'])
vip_pos_x = np.array(data['vip_pos_x'])
vip_pos_y = np.array(data['vip_pos_y'])
time = np.array(data['time'])


time_steps = np.arange(vip_rho.size)
dr = np.vstack((episode,done_reason))
dr_seperated= []
pos_seperated= []
time_seperated= []
for i in np.arange(1,100):
    dr_seperated += [done_reason[np.where(episode== i) ][-1]]
    pos_seperated += [list(zip(robot_pos_x[np.where(episode== i) ],robot_pos_y[np.where(episode== i) ] ))]
    time_seperated += [time[np.where(episode== i) ][-1]-time[np.where(episode== i) ][0]]

path = np.array(pos_seperated[0]) 
lengths = np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1)) # Length between corners
total_length = np.sum(lengths)
print(np.average(total_length))
print(np.average(time_seperated))
print(dr_seperated.count(0),dr_seperated.count(1),dr_seperated.count(2))


fig, ax = plt.subplots() 
data1 = [dr_seperated.count(2),85, 72, 43]
data2 = [dr_seperated.count(1), 35, 21, 16]
width =0.3
label= ['Agent raw','Agent with safety model','Agent wihout safety model','Agent complete'] 
plt.bar(label, data1, width=width,label='Sucess rate')
plt.bar(np.arange(len(data2))+ width, data2, width=width,label='Collision rate')
ax.legend(loc=1, prop={'size': 12}) 
plt.show()