import numpy as np 
import matplotlib.pyplot as plt
import csv
import os
import pandas as pd



path = 'evaluation/'

#make dir for plots
try:
    os.mkdir(path)
except OSError:
    print ("Creation of the directory %s failed" % path)
else:
    print ("Successfully created the directory %s " % path)


#read in evaluation data
data = pd.read_csv('evaluation.csv')
done_reason = np.array(data['done_reason'])
episode = np.array(data['episode'])
adult_robot_distance = np.array(data['nearest distance for adult'])
child_robot_distance = np.array(data['nearest distance for child'])
elder_robot_distance = np.array(data['nearest distance for elder'])
reward=np.array(data['reward'])
#read out some values
adult_counter = np.count_nonzero(done_reason == 3)
child_counter = np.count_nonzero(done_reason == 4)
elder_counter = np.count_nonzero(done_reason == 5)
time_out_counter = np.count_nonzero(done_reason == 0)
collision_counter = np.count_nonzero(done_reason == 1)
goal_counter = np.count_nonzero(ending ==2)
num_episodes = adult_counter + child_counter  + elder_counter+ time_out_counter + goal_counter + collision_counter

#check if there were humans in level
# human_exist = adult_robot_distance.size != 0

#read safty distance
safe_dist_adult= 0.8
safe_dist_child= 1.2
safe_dist_elder= 1.5
		
# max_step_time_out = max_time/time_step

#get time_to_goal and traveled distanse per episode
# time_to_goal = []
# traveled_distance = []
# frac_direct_traveled_dist = []

# for current_episode in range(num_episodes):
# 	idx_start = np.argwhere(episode == current_episode) if current_episode != 0 else 0
# 	idx_end = np.argwhere(episode == current_episode+1)
# 	if ending[idx_end] == 'goal':
# 		#correct robot position index mistake if episode_end != time_out
# 		if ending[idx_start] != 'time':
# 			idx_start = idx_start + 2 #if last episode was time then correction is NOT nessesary 
# 		idx_end = idx_end + 2 #must always be corrected if ending[idx_end] != time
			
# 		position_for_episode = robot_position[int(idx_start):int(idx_end)]
# 		position_for_episode = position_for_episode[~np.isnan(position_for_episode).any(axis=1)]

# 		time_to_goal.append(position_for_episode.shape[0] - 1)
# 		vector_array = position_for_episode[:-1] - position_for_episode[1:]
# 		distance = np.sum(np.sum(np.abs(vector_array)**2,axis=-1)**(1./2))
# 		traveled_distance.append(distance)
# 		#first episode goal info missing
# 		if current_episode != 0:
# 			frac_direct_traveled_dist.append(distance - goal_distance[current_episode-1])



#remove all nan 

# time_to_goal = np.array(time_to_goal)
dic={0 : adult_robot_distance, 1 : child_robot_distance, 2 :elder_robot_distance}
dic_safe={0 : safe_dist_adult, 1 : safe_dist_child, 2 :safe_dist_elder}
#hist plot of distances robot-human
for i in range(3):
    mu = np.mean(dic[i])
    std=np.std(dic[i])
    human_robot_distances = dic[i].flatten()
    counts, bins = np.histogram(human_robot_distances,bins=50)
    total_number = len(human_robot_distances)
    num_smaller_than_safety_distance = len(human_robot_distances[human_robot_distances < dic_safe[i]])

    plt.figure(i+1)
    plt.hist(bins[:-1],bins=bins,weights=counts/total_number*100,align = 'right',edgecolor='black', linewidth=0.8)
    plt.axvline(dic_safe[i], color='r', linestyle='dashed', linewidth=1.5)
    plt.annotate('safety distance = %.2f'%(dic_safe[i]) + '\n %.1f %% of all distances are smaller'%(num_smaller_than_safety_distance/total_number*100), xy=(0, 0.85), xycoords='axes fraction',color = 'red')
    plt.annotate('$\mu = %.2f$'%(mu) + ' \n$\sigma^2 = %.2f$'%(std), xy=(0.85, 0.85), xycoords='axes fraction',bbox=dict(boxstyle="round", fc="w"))
    plt.title('Hist of human distances')
    plt.xlabel('distance')
    plt.ylabel('relative counts [%]')
    plt.savefig(path+ 'human_distance_hist')
    plt.clf()
#plot distribution robot action 
# labels = 'forward', 'forward_left',  'forward_right', 'forward_strong_left', 'forward_strong_right', 'backward', 'stop'
# sizes = []
# actino_length = robot_action.shape
# for i in range(7):
# 	occurens_of_action = np.sum(i == robot_action)/actino_length
# 	sizes.append(float(occurens_of_action * 100)) #occurence in percent

# plt.figure(2)
# plt.pie(sizes, labels=labels, autopct='%1.1f%%',
#         shadow=False, startangle=90)
# plt.axis('equal')  # Equal aspect ratio ensures that pie is drawn as a circle.
# plt.title('distribution of robot actions in %')
# plt.savefig(path + 'action_pie')
# plt.clf()

# #box plot auf time to reach goal 
# plt.figure(3)
# #plt.subplot(2,2,1)
# #plt.title('Time for reaching the goal')
# #plt.ylabel('number of actions to reach the goal')
# #plt.boxplot(time_to_goal)
# #plt.subplot(2,1,1)
# plt.title(r'$\frac {number steps}{max number steps}$')
# plt.ylabel('relative time')
# plt.boxplot(time_to_goal/max_step_time_out)
# plt.savefig(path + 'goal__time_box')
# plt.clf()

# plt.figure(4)
# plt.subplot(1,2,1)
# plt.title('distance to reach goal')
# plt.ylabel('distance')
# plt.boxplot(traveled_distance)
# plt.subplot(1,2,2)
# plt.title(r'$real\_distance - direct\_distance$')
# plt.ylabel('relative distance')
# plt.boxplot(frac_direct_traveled_dist)
# plt.savefig(path + 'goal_distance_box')
# plt.clf()

#calculate and plot ending counters per bins and in total and write text file
bin_size = 100
# ending = ending[~pd.isnull(data['Ending'])] #remove all nan
# ending = ending[ending != 'reset'] # remove all reset
done_reason = np.reshape(done_reason,(-1,bin_size))# reshape in bins of 100

collision_bin = []
adult_counter_bin = []
child_counter_bin = []
elder_counter_bin = []
time_out_counter_bin = []
goal_counter_bin = []

for row in done_reason:
	collision_bin.append(np.count_nonzero(row == 1)/bin_size*100)
	adult_counter_bin.append(np.count_nonzero(row == 3)/bin_size*100)
    child_counter_bin.append(np.count_nonzero(row == 4)/bin_size*100)
    elder_counter_bin.append(np.count_nonzero(row == 5)/bin_size*100)
	time_out_counter_bin.append(np.count_nonzero(row == 0)/bin_size*100)
	goal_counter_bin.append(np.count_nonzero(row == 2)/bin_size*100)

#write text file
# text_file = open(path + "evaluation_stats.txt", "w")
# text_file.write('For the evaluation the robot did ' + str(num_episodes) + ' episodes \n')
# text_file.write('The robot reached ' + str(goal_counter) + ' the goal \n')
# if human_exist:
# 	text_file.write('The robot hit ' + str(human_counter) + ' times a human \n')
# else:
# 	text_file.write('no human in level \n')
# text_file.write('The robot hit ' + str(wall_counter) + ' times a wall \n')
# text_file.write('The robot didn\'t reach the goal in time for ' + str(time_out_counter) + ' times \n')
# text_file.write('---------------------------------------------------------------- \n')
# text_file.write('Success rate: ' + str(round(goal_counter/num_episodes*100,1)) + '%')
# text_file.write(', Variance: ' + str(round(np.std(goal_counter_bin),1)) + '%\n')
# text_file.write('Human hit rate: ' + str(round(human_counter/num_episodes*100,1)) + '%')
# text_file.write(', Variance: ' + str(round(np.std(human_counter_bin),1)) + '%\n')
# text_file.write('Wall hit rate: ' + str(round(wall_counter/num_episodes*100,1)) + '%')
# text_file.write(', Variance: ' + str(round(np.std(wall_counter_bin),1)) + '%\n')
# text_file.write('Timeout rate: ' + str(round(time_out_counter/num_episodes*100,1)) + '%')
# text_file.write(', Variance: ' + str(round(np.std(time_out_counter_bin),1)) + '%\n')
# text_file.write('Rate of distances smaller than the safety distance: ' + str(round((num_smaller_than_safety_distance/total_number*100),1)) + '%')
# text_file.close()
#plot
plt.figure(5)
plt.title('episode endings over bins with size ' + str(bin_size))
plt.ylabel('number of counted endings')
plt.xlabel('bin')
plt.plot(wall_counter_bin,label='collision')
plt.plot(adult_counter_bin,label='adult')
plt.plot(child_counter_bin,label='child')
plt.plot(elder_counter_bin,label='elder')
plt.plot(time_out_counter_bin,label='time out')
plt.plot(goal_counter_bin,label='goal')
plt.legend()
plt.savefig(path + 'endings')
plt.clf()
print('evaluation done')