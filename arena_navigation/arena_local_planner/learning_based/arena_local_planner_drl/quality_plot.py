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

def smooth(scalars, weight) :  # Weight between 0 and 1
    last = scalars[0]  # First value in the plot (first timestep)
    smoothed = list()
    for point in scalars:
        smoothed_val = last * weight + (1 - weight) * point  # Calculate smoothed value
        smoothed.append(smoothed_val)                        # Save it
        last = smoothed_val                                  # Anchor the last smoothed value
        
    return smoothed

path = 'evaluation_360/'

#make dir for plots
# try:
#     os.mkdir(path)
# except OSError:
#     print ("Creation of the directory %s failed" % path)
# else:
#     print ("Successfully created the directory %s " % path)


#read in evaluation data
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
vip_pos_y = smooth(vip_pos_y,0.9)

time_steps = np.arange(vip_rho.size)
# print(vip_velocity.size)

pathes = ['5Obs','10Obs','20Obs','crowd','running']
for i,l in enumerate(pathes):
    pathes[i] = 'data/requestingGuide/' + l +'/'

for path in pathes :

    data = pd.read_csv(path+'evaluation_360_4.csv')

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
    vip_pos_y = smooth(vip_pos_y,0.9)

    time_steps = np.arange(vip_rho.size)
    fig, ax = plt.subplots()  # Create a figure containing a single axes.
    ax.plot(time_steps,smooth(vip_rho,0.9 )  ,'-',color='tab:green',alpha=0.7,label='$SDRL_{Complete}$')
    # plt.axvline(x=np.where(task_flag==2)[0][0],label ='Tasks Seperation')
    # plt.title('Task1                                     Task2       ')
    # plt.axvline(x=np.where(task_flag==2)[0][0],color='black')
    # ax.text(np.where(task_flag==2)[0][0]+10, 2.5, 'Task Seperation',fontsize=8,rotation=270,color='black')

    plt.axvline(x=np.where(task_flag==4)[0][0],color='black')
    ax.text(np.where(task_flag==4)[0][0]+10, 10, 'Task Seperation',rotation=270,color='black',fontsize=10)

    plt.axvline(x=np.where(task_flag==5)[0][0],color='black')
    ax.text(np.where(task_flag==5)[0][0]+10, 10, 'Task Seperation',rotation=270,color='black',fontsize=10)

    plt.title( '                      Task1                                 Task 4                    Task5       ')
    plt.ylabel('Distance to Vip (m)',fontsize=14)
    plt.xlabel('Time (s)',fontsize=14)


    plt.axhline(y=4.0 ,linestyle='--',color='red')
    ax.text(20, 4.4, 'Max Distance Thresthold',fontsize=8,color='tab:red')
    fig.canvas.draw()

    fig.tight_layout()
    plt.axis([0, time_steps.size, 2.0, 18.0])
    ax.legend(loc=1, prop={'size': 14}) 
    plt.xticks(fontsize=12 ) 
    plt.yticks(fontsize=12 ) 


    plt.savefig(path+'Distance_To_Vip.png')

    plt.show(block=False)

    fig, ax = plt.subplots()  # Create a figure containing a single axes.

    ax.plot(time_steps,smooth(vip_velocity,.9) ,'--',label ='$VIP_{Complete}$ ',color='tab:green',alpha=0.7)
    ax.plot(time_steps, smooth(robot_velocity ,.9),'-',label='$SDRL_{Complete}$',color='tab:green',alpha=0.7)

    plt.axvline(x=np.where(task_flag==4)[0][0],color='black')
    # ax.text(np.where(task_flag==4)[0][0]+10, 14, 'Task Seperation',fontsize=7,rotation=270,color='black')

    plt.axvline(x=np.where(task_flag==5)[0][0],color='black')
    # ax.text(np.where(task_flag==5)[0][0]+10, 14, 'Task Seperation',fontsize=7,rotation=270,color='black')

    plt.title('                      Task1                                 Task 4                    Task5       ')

    plt.ylabel('Velocitys (m/s) ',fontsize=14)
    plt.xlabel('Time (s)',fontsize=14)
    fig.canvas.draw()
    fig.tight_layout()

    plt.axis([0, time_steps.size, 0.0, 4.0])
    ax.legend(loc=2, prop={'size': 14}) 
    # 
    plt.xticks(fontsize=12 ) 
    plt.yticks(fontsize=12 ) 

  
    plt.savefig(path+'Vip-Velocity.png')
    plt.show(block=False)

    fig, ax = plt.subplots()  # Create a figure containing a single axes.
    ax.plot(time_steps,smooth( np.absolute(robot_orientation - vip_orientation),.9) ,'o',color='tab:green',alpha=0.7)
    # plt.axvline(x=np.where(task_flag==4)[0][0],label ='Tasks Seperation')
    # plt.axvline(x=np.where(task_flag==5)[0][0])
    # plt.title('                      Task1                                 Task 4                    Task5       ')

    # plt.axvline(x=np.where(task_flag==2)[0][0],label ='Tasks Seperation')
    # plt.title('Task1                                     Task2       ')
    plt.ylabel('Orientation Difference (°)')
    plt.xlabel('Time Steps (s)')
    plt.axis([0, vip_rho.size, -1.0,7.0])
    # ax.legend(loc=1, prop={'size': 8}) 


    plt.xticks(fontsize=12 ) 
    plt.yticks(fontsize=12 ) 

    # plt.savefig(path+'Orientation_Difference.png')
    plt.show(block=False)


    w = 4
    h = 3
    d = 70
    plt.figure(figsize=(w, h), dpi=d)
    fig, ax = plt.subplots()

    # plt.grid()
    ax.axis([0, 25, 0, 20])
    string_path_data =  [] 
    plt.xlabel('X Coordinates',fontsize=14)
    plt.ylabel('Y Coordinates',fontsize=14)

    for i in range(len(robot_pos_x)) :
        x = robot_pos_x[i]
        y = robot_pos_y[i]
        
        if i == 0:
            string_path_data = string_path_data +[(mpath.Path.MOVETO,(x,y))]
            ax.text(x, y+0.2, '  start 0',color='tab:green',fontstyle='oblique',fontsize=10)
            x_start = x
            y_start = y
            plt.scatter([x], [y],color='tab:green')


        elif  i == len(robot_pos_x) -1 : 
            string_path_data = string_path_data +[(mpath.Path.STOP,(x,y))]
            ax.text(x, y+0.2, 'End                      ',color='tab:green',fontstyle='oblique',fontsize=10)


        else :
            string_path_data = string_path_data +[(mpath.Path.LINETO,(x,y))]
            

        if i % 50 == 0 and i > 0 and (np.absolute(x_start - x )> 0.5 and np.absolute(y_start - y) > 0.5 ) :
            
            ax.text(x, y+0.2, i,color='tab:green',fontstyle='oblique',verticalalignment='bottom' ,fontsize=10)
            plt.scatter([x], [y],color='tab:green')



    codes, verts = zip(*string_path_data)
    string_path = mpath.Path(verts, codes)
    patch = mpatches.PathPatch(string_path, facecolor="none", lw=2)



    fap1 = mpatches.FancyArrowPatch(path=string_path,
                                    arrowstyle="-|>,head_length=10,head_width=5")
    # ax.add_patch(fap1)
    xs, ys = zip(*verts)
    ax.plot(xs, ys, '--', lw=2, color='tab:green', ms=2,label ='$SDRL_{Complete}$')
    # plt.show()
    xs, ys

    string_path_data =  [] 
    x_start = -1
    y_start = -1

    for i in range(len(robot_pos_x)) :
        x = vip_pos_x[i]
        y = vip_pos_y[i]
        
        if i == 0:
            string_path_data = string_path_data +[(mpath.Path.MOVETO,(x,y))]
            ax.text(x, y+0.2, '  Start 0 ',color='tab:green',fontstyle='oblique',fontsize=10)
            x_start = x
            y_start = y
            plt.scatter([x], [y],color='tab:green')

        elif  i == len(robot_pos_x) -1 : 
            string_path_data = string_path_data +[(mpath.Path.STOP,(x,y))]
            # ax.text(x, y, i,color='red')
            ax.text(x, y+0.2, 'End                      ',color='tab:green',fontstyle='oblique',fontsize=10)
        else :
            string_path_data = string_path_data +[(mpath.Path.LINETO,(x,y))]
            

        if i % 50 == 0 and i > 0 and (np.absolute(x_start - x )> 0.5 and np.absolute(y_start - y) > 0.5 )  :
            
            ax.text(x, y+0.2, i,color='tab:green',fontstyle='oblique',fontsize=10)
            plt.scatter([x], [y],color='tab:green')


    codes, verts = zip(*string_path_data)



    string_path = mpath.Path(verts, codes)
    patch = mpatches.PathPatch(string_path, facecolor="none", lw=2)


    fap1 = mpatches.FancyArrowPatch(path=string_path,
                                    arrowstyle="-|>,head_length=10,head_width=5",color='red')
    # ax.add_patch(fap1)

    xs, ys = zip(*verts)
    ax.plot(xs, ys, '-', lw=2, color='tab:green', ms=2,label ='$VIP_{Complete}$')

    

    color =['#e6194b', '#3cb44b', '#ffe119', '#4363d8', '#f58231', '#911eb4', '#46f0f0', '#f032e6', '#bcf60c', '#fabebe', '#008080', '#e6beff',
    '#9a6324', '#fffac8', '#800000', '#aaffc3', '#808000', '#ffd8b1', '#000075', '#808080', '#ffffff', '#000000']

    for j in np.arange(1,20):
        string_path_data =  [] 
        x_start = -1
        y_start = -1

        for i in range(len(robot_pos_x)) :
            x = np.array(data['ped_pos_x'+str(j)])[i]
            y = np.array(data['ped_pos_y'+str(j)])[i]
            state = np.array(data['ped_behavior'+str(j)])[i]
            
            if i == 0:
                string_path_data = string_path_data +[(mpath.Path.MOVETO,(x,y))]

            elif  i == len(robot_pos_x) -1 : 
                string_path_data = string_path_data +[(mpath.Path.STOP,(x,y))]
            

            else :
                string_path_data = string_path_data +[(mpath.Path.LINETO,(x,y))]
                
    


        codes, verts = zip(*string_path_data)



        string_path = mpath.Path(verts, codes)
        # patch = mpatches.PathPatch(string_path, facecolor="none", lw=2)


        # # fap1 = mpatches.FancyArrowPatch(path=string_path,
        # #                                 arrowstyle="-|>,head_length=10,head_width=5",color='black')
        # # ax.add_patch(fap1)
        # ax.add_patch(patch) 

        xs, ys = zip(*verts)
        if j ==1 :
            ax.plot(xs, ys, '--', lw=2, color='tab:brown',alpha= 0.3, ms=3,label ='Obstacle ')
        else :
            ax.plot(xs, ys, '--', lw=2, color='tab:brown',alpha= 0.3, ms=3)


    for j in np.arange(1,20):
        string_path_data =  [] 
        x_start = -1
        y_start = -1

        for i in range(len(robot_pos_x)) :
            x = np.array(data['ped_pos_x'+str(j)])[i]
            y = np.array(data['ped_pos_y'+str(j)])[i]
            state = np.array(data['ped_behavior'+str(j)])[i]

            
            if i == 0:
                ax.text(x, y+0.2, i,color='tab:brown',alpha= 0.7,fontstyle='oblique',fontsize=10)
                x_start = x
                y_start = y
                plt.scatter([x], [y],color='tab:brown',alpha= 0.3)

            elif  i == len(robot_pos_x) -1 : 
                # ax.text(x, y, i,color='red')
                ax.text(x, y+0.2, i,color='tab:brown',alpha= 0.7,fontstyle='oblique',fontsize=10)
                plt.scatter([x], [y],color='tab:brown',alpha= 0.3)

            # ax.plot(xs, ys, '--', lw=2, color=color[j], ms=3,label ='obstacle path')

    


            if i % 200 == 0 and i > 0 and (np.absolute(x_start - x )> 0.1 and np.absolute(y_start - y) > 0.1 )  :
                
                plt.scatter([x], [y],color='tab:green',alpha= 0.3)

                # ax.text(x, y, i,color='green',alpha= 0.7)
                

        ax.legend(loc=1, prop={'size': 14}) 


    fig.canvas.draw()
    fig.tight_layout()
    plt.xticks(fontsize=12 ) 
    plt.yticks(fontsize=12 ) 



    plt.savefig(path+'Trajectories.png')
    plt.show(block=False)

    print('evaluation done')