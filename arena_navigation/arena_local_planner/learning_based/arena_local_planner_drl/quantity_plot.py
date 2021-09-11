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

time_steps = np.arange(vip_rho.size)
dr = np.vstack((episode,done_reason))
dr_seperated= []
pos_seperated= []

for i in np.arange(1,35):
    dr_seperated += [done_reason[np.where(episode== i) ][-1]]
    pos_seperated += [list(zip(robot_pos_x[np.where(episode== i) ],robot_pos_y[np.where(episode== i) ] ))]

path = np.array(pos_seperated[0]) 
lengths = np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1)) # Length between corners
total_length = np.sum(lengths)
print(total_length)






