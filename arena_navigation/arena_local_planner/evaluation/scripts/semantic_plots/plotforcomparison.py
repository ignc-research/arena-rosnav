import numpy as np 
import matplotlib.pyplot as plt
import csv
import os
import pandas as pd
import seaborn as sns

sns.set_style("whitegrid")
blue, = sns.color_palette("muted", 1)


path = 'comparison/'

#make dir for plots
try:
    os.mkdir(path)
except OSError:
    print ("Creation of the directory %s failed" % path)
else:
    print ("Successfully created the directory %s " % path)

#the list of files for evaluations (should be in the same folder of the scripts)
#raw mode   # normal mode   #danger mode
file_list=['evaluation_raw.csv','evaluation_nz.csv','evaluation_dz.csv']

len_file=len(file_list)

# data=[None]*len_file
# done_reason=[None]*len_file
episode_length=[None]*len_file
adult_robot_distance=[None]*len_file 
child_robot_distance =[None]*len_file
elder_robot_distance = [None]*len_file
steps_exceed_adult = [None]*len_file
steps_exceed_child =[None]*len_file
steps_exceed_elder = [None]*len_file
total_time_steps = [None]*len_file
total_time = [None]*len_file
path_len = [None]*len_file
reward = [None]*len_file
#read out some values
adult_counter = [None]*len_file
child_counter = [None]*len_file
elder_counter = [None]*len_file
time_out_counter = [None]*len_file
collision_counter = [None]*len_file
goal_counter = [None]*len_file
num_episodes = [None]*len_file
success_rate = [None]*len_file
collision_rate=[None]*len_file
path_len=[None]*len_file

AET = [None]*len_file
# AET1 = np.mean(total_time1)
time_exceed_adult_average = [None]*len_file
time_exceed_child_average = [None]*len_file
time_exceed_elder_average = [None]*len_file
time_exceed_all_average = [None]*len_file
y=[None]*len_file

dist_adult_average = [None]*len_file
dist_child_average = [None]*len_file
dist_elder_average = [None]*len_file
dist_all_average = [None]*len_file

y_dist=[None]*len_file

for i,file in enumerate(file_list):
    data=pd.read_csv(file)
    done_reason = np.array(data['done_reason'])
    episode_length[i]=len(data['episode'])
    adult_robot_distance[i] = np.array(data['nearest distance for adult'])
    child_robot_distance[i] = np.array(data['nearest distance for child'])
    elder_robot_distance[i] = np.array(data['nearest distance for elder'])
    steps_exceed_adult[i] = np.array(data['time DExc for adult'])
    steps_exceed_child[i] = np.array(data['time DExc for child'])
    steps_exceed_elder[i] = np.array(data['time DExc for elder'])
    total_time_steps[i] = np.array(data['total time steps'])
    total_time[i] = np.array(data['total time'])
    path_len[i] = np.array(data['path'])
    reward[i] = np.array(data['reward'])
    #read out some values
    adult_counter[i] = np.count_nonzero(done_reason == 3)
    child_counter[i] = np.count_nonzero(done_reason == 4)
    elder_counter[i] = np.count_nonzero(done_reason == 5)
    time_out_counter[i] = np.count_nonzero(done_reason == 0)
    collision_counter[i] = np.count_nonzero(done_reason == 1)
    goal_counter[i] = np.count_nonzero(done_reason ==2)
    num_episodes[i] = adult_counter[i] + child_counter[i] + elder_counter[i] + time_out_counter[i] + goal_counter[i] + collision_counter[i]
    success_rate[i] = goal_counter[i]/num_episodes[i]*100
    collision_rate[i]=collision_counter[i]/num_episodes[i]*100
    path_len[i]=np.mean(path_len[i])

    AET[i]=np.mean(total_time[i])
    time_exceed_adult_average[i] = np.mean(steps_exceed_adult[i]/total_time_steps[i])*100
    time_exceed_child_average[i] = np.mean(steps_exceed_child[i]/total_time_steps[i])*100
    time_exceed_elder_average[i] = np.mean(steps_exceed_elder[i]/total_time_steps[i])*100
    time_exceed_all_average[i] = (time_exceed_adult_average[i]+time_exceed_child_average[i]+time_exceed_elder_average[i])/3
    y[i]=np.array([time_exceed_adult_average[i],time_exceed_child_average[i],time_exceed_elder_average[i],time_exceed_all_average[i]]).reshape(-1,1)


    dist_adult_average[i] = np.mean(adult_robot_distance[i])
    dist_child_average[i] = np.mean(child_robot_distance[i])
    dist_elder_average[i] = np.mean(elder_robot_distance[i])
    dist_all_average[i] = (dist_adult_average[i]+dist_child_average[i]+dist_elder_average[i])/3

    y_dist[i]=np.array([dist_adult_average[i],dist_child_average[i],dist_elder_average[i],dist_all_average[i]]).reshape(-1,1)


y=np.concatenate(y,axis=1)
y_dist=np.concatenate(y_dist,axis=1)

#read safety distance
safe_dist_adult= 1.0
safe_dist_child= 1.2
safe_dist_elder= 1.5

#bar plot for time exD.
# x=[1,2]
x_labels = ['Adult','Child','Elder','Average']
class_color = ['tab:red','tab:blue','tab:green','tab:orange']
x=np.arange(len_file)
width = 0.1
fig,ax = plt.subplots()
ax.grid('on')
for i in range(len(x_labels)):
    ax.bar(x+i*width,y[i],width,alpha=0.9,color=class_color[i])
    
plt.annotate('$AT_{raw} = %.2fs$'%(AET[0]) + ' \n$AT_{sz} = %.2fs$'%(AET[1])+' \n$AT_{dz} = %.2fs$'%(AET[2]), xy=(0.4, 0.8), xycoords='axes fraction',bbox=dict(boxstyle="round", fc="w",edgecolor='black'))
plt.legend(x_labels, loc="best")
ax.set_xticks(x+1.5*width)
ax.set_xticklabels(['Raw','Static Zone','Dynamic Zone'])
# ax.set_xticks([x+width])
# ax.set_xticklabels(x_labels)
plt.title('Average Time Within Safety Zone',fontsize=16, fontweight='bold')
# plt.xlabel('time/s')
plt.ylabel('Time [%]', fontsize=16)
plt.xlabel('Agent Mode', fontsize=16)
plt.savefig(path + 'bar_chart_exceeding_counts_about_Time')
plt.clf()


#bar plot for dist
# x=[1,2]
x_labels = ['Adult','Child','Elder','Average']
class_color = ['tab:red','tab:blue','tab:green','tab:orange']
x=np.arange(len_file)
width = 0.1
fig,ax = plt.subplots()

for i in range(len(x_labels)):
    ax.bar(x+i*width,y_dist[i],width,alpha=0.7,color=class_color[i])
    """[summary]
    """
plt.annotate('$SR_{raw} = %.2f$'%(success_rate[0]) + ' $CR_{raw} = %.2f$'%(collision_rate[0])+'\n$SR_{sz} = %.2f$'%(success_rate[1]) + ' $CR_{sz} = %.2f$'%(collision_rate[1])+'\n$SR_{dz} = %.2f$'%(success_rate[2]) + ' $CR_{dz} = %.2f$'%(collision_rate[2]), xy=(0.6, 0.05), xycoords='axes fraction',bbox=dict(boxstyle="round", fc="w",edgecolor='black'))
# plt.annotate('$sr1 = %.2f$'%(success_rate1) + ' \n$cr1 = %.2f$'%(collision_rate1), xy=(0.3, 0.85), xycoords='axes fraction',bbox=dict(boxstyle="round", fc="w"))
plt.legend(x_labels,framealpha=0.4)
ax.set_xticks(x+1.5*width)
ax.set_xticklabels(['Raw','Static Zone','Dynamic Zone'])
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.title('Average Distance To Each Obstacle Type', fontsize=20, fontweight='bold')
plt.ylabel('Distance [m]', fontsize=16)
plt.xlabel('Agent Mode', fontsize=16)
plt.savefig(path + 'average_distance')
plt.clf()


# time_to_goal = np.array(time_to_goal)
dic={0 : adult_robot_distance, 1 : child_robot_distance, 2 :elder_robot_distance}
dic_safe={0 : safe_dist_adult, 1 : safe_dist_child, 2 :safe_dist_elder}
dic_h={0 : 'Adult', 1 : 'Child', 2 :'Elder'}
mode ={0:'Raw',1:'Static Zone',2:'Dynamic Zone'}
#hist plot of distances robot-human
for j in range(len_file):
    for i in range(3):
        mu = np.mean(dic[i][j])
        std=np.std(dic[i][j])
        human_robot_distances = dic[i][j].flatten()
        counts, bins = np.histogram(human_robot_distances,bins=50)
        total_number = len(human_robot_distances)
        num_smaller_than_safety_distance = len(human_robot_distances[human_robot_distances < dic_safe[i]])

        plt.figure(i+j+1)
        plt.hist(bins[:-1],bins=bins,weights=counts/total_number*100, align = 'right',edgecolor='black', linewidth=0.2, color=blue)
        plt.axvline(dic_safe[i], color='r', linestyle='dashed', linewidth=1.5)
        plt.annotate('Safety distance = %.2f'%(dic_safe[i]) + '\n %.1f %% of all distances are smaller'%(num_smaller_than_safety_distance/total_number*100), xy=(0.35, 0.86), xycoords='axes fraction',fontsize=14, color = 'red', bbox=dict(boxstyle="round", fc="w",edgecolor='red'))
        plt.annotate('$\mu = %.2f$'%(mu) + ' \n$\sigma^2 = %.2f$'%(std), xy=(0.85, 0.85), xycoords='axes fraction',fontsize=14, bbox=dict(boxstyle="round", fc="w",edgecolor='black'))
        plt.title('Distances Of Closest '+ dic_h[i] +' To Robot - '+mode[j]+' Mode', fontsize=16, fontweight='bold')
        plt.xlabel('Distance [m]', fontsize=16)
        plt.ylabel('Relative Counts [%]', fontsize=16)
        plt.xlim([0, 8])
        plt.ylim([0, 20])
        plt.xticks(fontsize=14)
        plt.yticks(fontsize=14)
        plt.savefig(path+dic_h[i]+'_distance_hist_'+mode[j])
        plt.clf()

print('evaluation done')