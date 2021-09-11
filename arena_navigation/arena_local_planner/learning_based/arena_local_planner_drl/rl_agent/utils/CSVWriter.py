import csv
import numpy as np
import rospy
# 'w' = write mode
class CSVWriter():
    def __init__(self):
        # create evaluation.csv file
        peds = []
        num_obs= [0,0,5,10,20,20]

        for i in np.arange(1,num_obs[rospy.get_param("/curr_stage", -1)]) :
            peds += ['ped_pos_x'+str(i)]
            peds += ['ped_pos_y'+str(i)]
            peds += ['ped_behavior'+str(i)]


        with open('evaluation.csv', 'w+', newline = '') as fp:
            a = csv.writer(fp, delimiter = ',')
            data = [['episode','task_flag','done_reason','time', 'nearest distance for adult','nearest distance for child','nearest distance for elder',
            'nearest distance for forklift','nearest distance for servicerobot','nearest distance for random_wanderer','reward',
            'safe distance adult','safe distance child','safe distance elder','safe distance forklift','safe distance servicerobot' 
            ,'safe distance random_wanderer','vip_velocity','robot_velocity','vip_orientation','robot_orientation'
            ,'vip_rho','robot_pos_x','robot_pos_y','vip_pos_x','vip_pos_y']+peds]
            a.writerows(data)

    def addData(self, data:np.array):
        #add new data to the file 
        with open('evaluation.csv', 'a+', newline = '') as fp:
            a = csv.writer(fp, delimiter = ',')
            data =data.reshape(1,-1)
            a.writerows(data)
