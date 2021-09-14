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
# data = pd.read_csv('guidingHuman/evaluation_360.csv')
# done_reason = np.array(data['done_reason'])
# episode = np.array(data['episode'])
# task_flag = np.array(data['task_flag'])
# vip_rho = np.array(data['vip_rho'] ) 
# robot_orientation = np.array(data['robot_orientation'])
# vip_orientation = np.array(data['vip_orientation'])
# robot_velocity = np.array(data['robot_velocity'])
# vip_velocity = np.array(data['vip_velocity'])
# robot_pos_x = np.array(data['robot_pos_x'])
# robot_pos_y = np.array(data['robot_pos_y'])
# vip_pos_x = np.array(data['vip_pos_x'])
# vip_pos_y = np.array(data['vip_pos_y'])
# vip_pos_y = smooth(vip_pos_y,0.9)
# robot_pos_y = smooth(vip_pos_y,0.9)


pathes = ['5Obs','10Obs','20Obs','crowd','running']
for i,l in enumerate(pathes):
    pathes[i] = 'data/normal/' + l +'/'

for path in pathes :
    data = pd.read_csv(path+'evaluation_360_1.csv')

    fig, ax = plt.subplots()  # Create a figure containing a single axes.
    fig.canvas.draw()
    fig.tight_layout()
    vip_rho = np.array(data['vip_rho'] ) 
    vip_rho = smooth(vip_rho,0.95)
    task_flag = np.array(data['task_flag'])
    time_steps = np.arange(len(task_flag)) 



    plt.axhline(y=4.0 ,linestyle='--',color='red')
    ax.text(20, 4.4, 'Max Distance Thresthold',fontsize=10,color='tab:red')

    ax.plot(time_steps,vip_rho,'-',color='tab:purple',alpha=0.9,label ='$SDRL_{raw}$')
    plt.plot([len(time_steps)], [vip_rho[-1]], marker='x', markersize=7, color="tab:purple")


    data = pd.read_csv(path+'evaluation_360_2.csv')

    vip_rho = np.array(data['vip_rho'] ) 
    vip_rho = smooth(vip_rho,0.95)
    task_flag = np.array(data['task_flag'])
    time_steps = np.arange(len(task_flag)) 

    ax.plot(time_steps,vip_rho,'-',color='tab:orange',alpha=0.9,label ='$SDRL_{SafeZone}$')
    plt.plot([len(time_steps)], [vip_rho[-1]], marker='x', markersize=7, color="tab:orange")



    data = pd.read_csv(path+'evaluation_360_3.csv')

    vip_rho = np.array(data['vip_rho'] ) 
    vip_rho = smooth(vip_rho,0.95)
    task_flag = np.array(data['task_flag'])
    time_steps = np.arange(len(task_flag)) 

    ax.plot(time_steps,vip_rho,'-',color='tab:blue',alpha=0.9,label ='$SDRL_{NoSafeZone}$')
    plt.plot([len(time_steps)], [vip_rho[-1]], marker='x', markersize=7, color="tab:blue")

    data = pd.read_csv(path+'evaluation_360_4.csv')

    vip_rho = np.array(data['vip_rho'] ) 
    vip_rho = smooth(vip_rho,0.95)
    task_flag = np.array(data['task_flag'])
    time_steps = np.arange(len(task_flag)) 


    ax.plot(time_steps,vip_rho,'-',color='tab:green',alpha=0.9,label ='$SDRL_{Complete}$')
    plt.plot([len(time_steps)], [vip_rho[-1]], marker='x', markersize=7, color="tab:green")


    plt.title('Task1                                     Task2       ')

    # plt.axvline(x=np.where(task_flag==4)[0][0],label ='Tasks Seperation')
    # plt.axvline(x=np.where(task_flag==5)[0][0])
    # plt.title('                      Task3                                 Task 4                    Task5       ')
    plt.ylabel('Distance to Vip (m)',fontsize=14)
    plt.xlabel('Time (s)',fontsize=14)

    plt.axis([0, time_steps.size, 1.0, 22.0])
    ax.legend(loc=1, prop={'size': 12} )
    plt.subplots_adjust(top=0.88,
    bottom=0.11,
    left=0.11,
    right=0.9,
    hspace=0.2,
    wspace=0.2)
    plt.xticks(fontsize=12 ) 
    plt.yticks(fontsize=12 ) 


    # plt.savefig(path+'Distance_To_Vip.png',dpi=300)

    plt.show(block=False)




    fig, ax = plt.subplots()  # Create a figure containing a single axes.

    data = pd.read_csv(path+'evaluation_360_1.csv')


    robot_velocity = np.array(data['robot_velocity'])
    vip_velocity = np.array(data['vip_velocity'])
    robot_velocity = smooth(robot_velocity ,.9)
    vip_velocity = smooth(vip_velocity,.9)

    task_flag = np.array(data['task_flag'])
    time_steps = np.arange(len(task_flag)) 



    ax.plot(time_steps, robot_velocity,'-',label='$SDRL_{raw}$',color='tab:purple',alpha=0.9)
    plt.plot([len( time_steps)], [robot_velocity[-1]], marker='x', markersize=7, color="tab:purple")

    data = pd.read_csv(path+'evaluation_360_2.csv')


    robot_velocity = np.array(data['robot_velocity'])
    vip_velocity = np.array(data['vip_velocity'])
    robot_velocity = smooth(robot_velocity ,.9)
    vip_velocity = smooth(vip_velocity,.9)
    task_flag = np.array(data['task_flag'])
    time_steps = np.arange(len(task_flag)) 


    ax.plot(time_steps, robot_velocity,'-',label='$SDRL_{SafeZone}$',color='tab:orange',alpha=0.9)
    plt.plot([len( time_steps)], [robot_velocity[-1]], marker='x', markersize=7, color="tab:orange")

    data = pd.read_csv(path+'evaluation_360_3.csv')

    robot_velocity = np.array(data['robot_velocity'])
    vip_velocity = np.array(data['vip_velocity'])
    robot_velocity = smooth(robot_velocity ,.9)
    vip_velocity = smooth(vip_velocity,.9)
    task_flag = np.array(data['task_flag'])
    time_steps = np.arange(len(task_flag)) 


    ax.plot(time_steps,robot_velocity ,'-',label='$SDRL_{NoSafeZone}$',color='tab:blue',alpha=0.9)
    plt.plot([len( time_steps)], [robot_velocity[-1]], marker='x', markersize=6, color="tab:blue")

    data = pd.read_csv(path+'evaluation_360_4.csv')

    robot_velocity = np.array(data['robot_velocity'])
    vip_velocity = np.array(data['vip_velocity'])
    task_flag = np.array(data['task_flag'])
    robot_velocity = smooth(robot_velocity ,.9)
    vip_velocity = smooth(vip_velocity,.9)
    task_flag = np.array(data['task_flag'])
    time_steps = np.arange(len(task_flag)) 


    ax.plot(time_steps, robot_velocity ,'-',label='$SDRL_{Complete}$',color='tab:green',alpha=0.9)
    plt.plot([len( time_steps)], [robot_velocity[-1]], marker='x', markersize=6, color="tab:green")

    plt.ylabel('Velocitys (m/s) ',fontsize=14)
    plt.xlabel('Time (s)',fontsize=14)

    fig.canvas.draw()
    fig.tight_layout()
    plt.axis([0, time_steps.size ,-1.0, 4.0])

    ax.legend(loc=2, prop={'size': 12}) 
    plt.xticks(fontsize=12 ) 
    plt.yticks(fontsize=12 ) 


    plt.savefig(path+'Vip-Velocity.png')
    plt.show(block=False)


    plt.figure(figsize=(4, 3), dpi=70)
    fig, ax = plt.subplots()

    color=['tab:purple','tab:orange','tab:blue','tab:green']
    colorVip=['purple','orange','blue','green']

    label= [' $SDRL_{raw}$','$SDRL_{SafeZone}$','$SDRL_{NoSafeZone}$','$SDRL_{Complete}$'] 



    for j in range(4):


        data = pd.read_csv(path+'evaluation_360_'+str(j+1)+'.csv')

        robot_pos_x = np.array(data['robot_pos_x'])
        robot_pos_y = np.array(data['robot_pos_y'])
        vip_pos_x = np.array(data['vip_pos_x'])
        vip_pos_y = np.array(data['vip_pos_y'])


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
                ax.text(x, y+0.2, '  start 0',color=color[j],fontstyle='oblique',alpha=0.9,fontsize=10)
                x_start = x
                y_start = y
                plt.scatter([x], [y],color=color[j],alpha=0.9)


            elif  i == len(robot_pos_x) -1 : 
                string_path_data = string_path_data +[(mpath.Path.STOP,(x,y))]
                ax.text(x, y+0.2, 'End                      ',fontsize=10,color=color[j],fontstyle='oblique',alpha=0.9)


            else :
                string_path_data = string_path_data +[(mpath.Path.LINETO,(x,y))]
                

            if i % 50 == 0 :
                
                ax.text(x, y+0.2, i,color=color[j],fontstyle='oblique',verticalalignment='bottom',alpha=0.9,fontsize=10)
                plt.scatter([x], [y],color=color[j],alpha=0.9)




        codes, verts = zip(*string_path_data)
        string_path = mpath.Path(verts, codes)
        patch = mpatches.PathPatch(string_path, facecolor="none", lw=2)



        # fap1 = mpatches.FancyArrowPatch(path=string_path,
        #                                 arrowstyle="-|>,head_length=10,head_width=5")l
        # ax.add_patch(fap1)
        xs, ys = zip(*verts)
        ax.plot(xs, ys, '-', lw=2, color=color[j], ms=8,label =label[j],alpha= 1.0)
        # plt.show()

    

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
            ax.plot(xs, ys, '--', lw=2, color='black',alpha= 0.3, ms=3,label ='obstacle')
        else :
            ax.plot(xs, ys, '--', lw=2, color='black',alpha= 0.3, ms=3)


    for j in np.arange(1,20):
        string_path_data =  [] 
        x_start = -1
        y_start = -1

        for i in range(len(robot_pos_x)) :
            x = smooth(np.array(data['ped_pos_x'+str(j)]),0.1)[i]
            y = smooth(np.array(data['ped_pos_y'+str(j)]),0.1)[i]
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
                
                plt.scatter([x], [y],color='tab:brown',alpha= 0.3)

                # ax.text(x, y, i,color='green',alpha= 0.7)
                

    ax.legend(loc=2, prop={'size': 12})

    fig.canvas.draw()
    fig.tight_layout()

    plt.xticks(fontsize=12 ) 
    plt.yticks(fontsize=12 ) 


    plt.savefig(path+'Trajectories.png')
    plt.show(block=False)

    print('evaluation done')