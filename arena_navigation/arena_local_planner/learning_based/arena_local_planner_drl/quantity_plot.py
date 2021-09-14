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


pathes = ['5Obs','10Obs','20Obs']
for i,l in enumerate(pathes):
    pathes[i] = 'data/quantity_normal/' + l +'/'


dr_seperated_list_list = []
pos_seperated_list_list = []
time_seperated_list_list = []
for path in pathes :
    dr_seperated_list = []
    pos_seperated_list = []
    time_seperated_list = []

    for i in np.arange(1,5):
        data = pd.read_csv(path+'evaluation_360_'+str(i)+'.csv')

        done_reason = np.array(data['done_reason'])
        episode = np.array(data['episode'])
        robot_pos_x = np.array(data['robot_pos_x'])
        robot_pos_y = np.array(data['robot_pos_y'])
        time = np.array(data['time'])

        dr = np.vstack((episode,done_reason))
        dr_seperated= []
        pos_seperated= []
        time_seperated= []
        for i in np.arange(1,101):
            dr_seperated += [done_reason[np.where(episode== i) ][-1]]
            pos_seperated += [list(zip(robot_pos_x[np.where(episode== i) ],robot_pos_y[np.where(episode== i) ] ))]
            time_seperated += [time[np.where(episode== i) ][-1]-time[np.where(episode== i) ][0]]
        dr_seperated_list += [dr_seperated]
        pos_seperated_list+= [pos_seperated]
        time_seperated_list+= [np.average(time_seperated)]

    dr_seperated_list_list += [dr_seperated_list]
    pos_seperated_list_list += [pos_seperated_list]
    time_seperated_list_list += [time_seperated_list]

fig, ax = plt.subplots() 
success = []
collsion = []
time_out = []
time_out_plot = []

plt.title('Agents success and failure over number of obstacles',fontweight="bold" ,fontsize=11)
for dr_seperated_list in dr_seperated_list_list :
    for i in range(4) :
        success += [dr_seperated_list[i].count(2)]
        if  dr_seperated_list[i].count(0)> 4:
            time_out += [dr_seperated_list[i].count(0)]
            collsion += [dr_seperated_list[i].count(1)]
            time_out_plot += [dr_seperated_list[i].count(0)+dr_seperated_list[i].count(1)]
        else:
            time_out += [0]
            collsion += [dr_seperated_list[i].count(1)+dr_seperated_list[i].count(0)]
            time_out_plot += [0]

label1= [' $SDRL_{raw}$','$SDRL_{SafeZone}$','$SDRL_{NoSafeZone}$','$SDRL_{Complete}$'] 
label2= [' $SDRL_{raw}$ ','$SDRL_{SafeZone}$ ','$SDRL_{NoSafeZone}$ ','$SDRL_{Complete}$ '] 
label3= [' $SDRL_{raw}$  ','$SDRL_{SafeZone}$  ','$SDRL_{NoSafeZone}$  ','$SDRL_{Complete}$  '] 
labels =label1+label2+label3

color1= ['tab:purple','tab:orange','tab:blue','tab:green'] 
color2= ['tab:purple','tab:orange','tab:blue','tab:green'] 
color3= ['tab:purple','tab:orange','tab:blue','tab:green'] 

colors =color1+color2+color3


xs = [0,1,2,3,5,6,7,8,10,11,12,13]
plt.bar(xs, success,label='Success rate',width=0.8,alpha= 0.9,color='tab:green')
plt.bar(xs, time_out_plot ,  bottom=success,label='Time out rate',width=0.8,alpha= 0.9,color='tab:orange')
plt.bar(xs, collsion,  bottom=success,label='Collision rate',width=0.8,alpha= 0.9,color='tab:red')
plt.xticks(xs, labels, fontsize = 12)
# plt.xlabel('5Obs                      10Obs                      20Obs',fontsize=12)
ax.annotate('5 Obs', ( 1.5, 30), ha='center',fontsize=13)
ax.annotate('10 Obs', ( 6.5, 30), ha='center',fontsize=13)
ax.annotate('20 Obs', ( 11.5, 30), ha='center',fontsize=13)

for tick in ax.get_xticklabels():
    tick.set_rotation(45)

ax.legend(loc=3, prop={'size': 14}) 
for i in range(12) :
    ax.annotate(f'{success[i] /100:.0%}', ( xs[i], success[i]-4), ha='center',fontsize=7)
    ax.annotate(f'{collsion[i] /100:.0%}', ( xs[i],success[i] + collsion[i]-4), ha='center',fontsize=7)
    if time_out[i] > 0: 
        ax.annotate(f'{time_out[i] /100:.0%}', ( xs[i],success[i] + time_out_plot[i]-4), ha='center',fontsize=7)
# ax.grid()
plt.ylabel('Success and failure in (%) ',fontsize=14)

fig.savefig('done_reason.png',dpi=300, format='png', bbox_inches='tight')



fig, ax = plt.subplots() 

avg_lens = []

for pos_seperated_list in pos_seperated_list_list :
    for pos_seperated in  pos_seperated_list:

        total_length = 0
        for l in pos_seperated :
            p = np.array(l) 
            lengths = np.sqrt(np.sum(np.diff(p, axis=0)**2, axis=1)) # Length between corners
            total_length += np.sum(lengths)

        avg_lens += [total_length/100]

fig, ax = plt.subplots() 




plt.xticks(xs, labels, fontsize = 12)
ax.annotate('5 Obs', ( 1.5, 9), ha='center',fontsize=14)
ax.annotate('10 Obs', ( 6.5, 9), ha='center',fontsize=14)
ax.annotate('20 Obs', ( 11.5,9 ), ha='center',fontsize=14)

# plt.xlabel('5 Obs                      10 Obs                      20 Obs',fontsize=12)
bar_list=plt.bar(xs, avg_lens,width=0.8,alpha= 0.9)
for i,bar in enumerate(bar_list) :
    bar.set_color(colors[i])
plt.title('Agents average path length over number of obstacles',fontweight="bold" ,fontsize=11)
for tick in ax.get_xticklabels():
    tick.set_rotation(45)
plt.ylabel('Average path length in (m) ',fontsize=14)

fig.savefig('avg_path_len.png', dpi=300, format='png', bbox_inches='tight')





fig, ax = plt.subplots() 
plt.title('Agents average time over number of obstacles',fontweight="bold" ,fontsize=11)
plt.xticks(xs, labels, fontsize = 11)
# plt.xlabel('5 Obs                      10 Obs                      20 Obs',fontsize=12)
times =[]
for time_seperated_list in time_seperated_list_list :
   for time_seperated in time_seperated_list : 
      times += [time_seperated]

bar_list=plt.bar(xs, times,label='Average time (Sec)',width=0.8,alpha=0.9)
for i,bar in enumerate(bar_list) :
    bar.set_color(colors[i])
ax.annotate('5 Obs', ( 1.5, 6), ha='center',fontsize=14)
ax.annotate('10 Obs', ( 6.5, 6), ha='center',fontsize=14)
ax.annotate('20 Obs', ( 11.5,6 ), ha='center',fontsize=14)
for tick in ax.get_xticklabels():
    tick.set_rotation(45)
plt.ylabel('Average time length in (sec) ',fontsize=11)


fig.savefig('avg_time.png', dpi=300, format='png', bbox_inches='tight')