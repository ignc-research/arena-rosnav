# 
import rosbag
import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.pyplot import figure
import seaborn as sb
import rospy 
from visualization_msgs.msg import Marker, MarkerArray


# 
class newBag():
    def __init__(self, planner,file_name, plot_style, bag_name, odom_topic="/sensorsim/police/odom", collision_topic="/sensorsim/police/collision"):
        # self.make_txt(file_name,"","w")
        self.bag = bagreader(bag_name)
        eps = self.split_runs(odom_topic, collision_topic)
        self.evalPath(plot_style,planner,file_name,eps)
        self.nc_total = 0

    def make_txt(self,file,msg,ron="a"):
        f = open(file, ron)
        f.write(msg)
        f.close()

    def make_heat_map(self,xy):
        plt.figure()
        data = np.zeros((25, 20))

        for xya in xy:
            for pos in xya:
                y = -int(round(pos[0], 0))
                x = int(round(pos[1], 0))
                data[x,y] += 1
                # print(data[x,y])
        heat_map = sb.heatmap(data,  cmap="YlGnBu")
        heat_map.invert_yaxis()
        plt.show()

    def split_runs(self,odom_topic,collision_topic):
        # get odometry
        
        odom_csv = self.bag.message_by_topic(odom_topic)
        df_odom = pd.read_csv(odom_csv, error_bad_lines=False)

        # get collision time
        try:
            collision_csv = self.bag.message_by_topic(collision_topic)
            df_collision = pd.read_csv(collision_csv, error_bad_lines=False)
        except Exception as e:
            df_collision = []
        t_col = []
        for i in range(len(df_collision)): 
            t_col.append(df_collision.loc[i, "Time"])
        self.nc_total = len(t_col)
        print("total #n of collisions: ",len(t_col))
        # get reset time
        reset_csv = self.bag.message_by_topic("/scenario_reset")
        df_reset = pd.read_csv(reset_csv, error_bad_lines=False)
        t_reset = []
        for i in range(len(df_reset)): 
            t_reset.append(df_reset.loc[i, "Time"])
        print("resets: ",len(t_reset))
        # print(t_reset)

        pose_x = []
        pose_y = []
        t = []
        bags = {}
        # run idx
        n = 0
        # collsion pos
        col_xy = []
        nc = 0


        for i in range(len(df_odom)): 
            current_time = df_odom.loc[i, "Time"]
            x = df_odom.loc[i, "pose.pose.position.x"]
            y = df_odom.loc[i, "pose.pose.position.y"]
            reset = t_reset[n]
            # check if respawned
            # if current_time > reset-6 and n < len(t_reset)-1 and x<0:
            global start_x
            if current_time > reset-6 and n < len(t_reset)-1 and x < start_x:
                n += 1
                # store the run
                bags["run_"+str(n)] = [pose_x,pose_y,t,col_xy]

                # reset 
                pose_x = []
                pose_y = []
                t = []
                col_xy = []
   
            if  len(pose_x) > 0:
                pose_x.append(x)
                pose_y.append(y)
            elif x < start_x:
                # print("run_"+str(n))
                # print("x:",x,"y",y)
                pose_x.append(x)
                pose_y.append(y)

            t.append(current_time)
            # get trajectory

            # check for col
            if len(t_col) > nc:
                if current_time >= t_col[nc]:
                    col_xy.append([x,y])
                    nc += 1

        if "run_1" in bags:    
            bags.pop("run_1")
        return bags
    
    def average(self,lst): 
        if len(lst)>0:
            return sum(lst) / len(lst) 
        else:
             return 0

    def print_patches(self,xya):
        global ax

        for run_a in xya:
            for col_xy in run_a:
                circle = plt.Circle((-col_xy[1], col_xy[0]), 0.3, color='r', fill = False)
                ax.add_patch(circle)

    def evalPath(self, clr, planner, file_name, bags = None):
        col_xy = []
        global ax
        # circle1 = plt.Circle((-9, 12), 0.3, color='r', fill = False)
        # ax.add_patch(circle1)
        durations = [] 
        trajs = []
        vels  = []

        self.make_txt(file_name, "\n"+"Evaluation of "+planner+":")
        
        for run in bags:
            if run != "nrun_30":
                pose_x = bags[run][0]
                pose_y = bags[run][1]

                x = np.array(pose_x)
                y = -np.array(pose_y)
                
                t = bags[run][2]

                dist_array = (x[:-1]-x[1:])**2 + (y[:-1]-y[1:])**2
                path_length = np.sum(np.sqrt(dist_array)) 
                # for av
                trajs.append(path_length)
                if path_length > 0:
                    ax.plot(y,x, clr, label='Cosine wave')

                duration = t[len(t)-1] - t[0]
                # for av
                durations.append(duration)
                av_vel = path_length/duration
                # for av
                vels.append(av_vel)

                n_col = len(bags[run][3])

                cr = run+": "+str([duration, path_length, av_vel, n_col])
                print(cr)
                self.make_txt(file_name, "\n"+cr)

                col_xy.append(bags[run][3])


        msg_planner = "\n----------------------   "+planner+" summary: ----------------------"
        msg_at = "\naverage time:        "+str(round(self.average(durations),3))+" s"
        msg_ap = "\naverage path length: "+str(round(self.average(trajs),3))+" m"
        msg_av = "\naverage velocity:    "+str(round(self.average(vels),3))+"  m/s"
        msg_col = "\ntotal number of collisions: "+str(self.nc_total)+"\n"

        print("----------------------   "+planner+"   ----------------------")
        print("average time:        ", round(self.average(durations),3), "s")
        print("average path length: ", round(self.average(trajs),3), " m")
        print("average velocity:    ", round(self.average(vels),3), "  m/s")

        self.make_txt(file_name,msg_planner)
        self.make_txt(file_name,msg_at)
        self.make_txt(file_name,msg_ap)
        self.make_txt(file_name,msg_av)
        self.make_txt(file_name,msg_col)

        self.print_patches(col_xy)
        # self.make_heat_map(col_xy)
            
def getMap(msg):
    global ax
    points_x = []
    points_y = []
    # print(msg.markers[0])
    for p in msg.markers[0].points:
        points_x.append(p.x-6)
        points_y.append(-p.y+6)
    plt.scatter(points_y, points_x)
    plt.show()

def run():
    global ax, start_x
    fig, ax = plt.subplots(figsize=(6, 12))



    # obstacle_map1_obs5.json
    start_x = 0.5
    fn = "obstacle_map1_obs5.txt"
    newBag("cadrl_01", fn, "b", "bags/scenaries/cadrl/cadrl_map1_ob5_vel_01.bag")
    newBag("cadrl_02", fn, "b", "bags/scenaries/cadrl/cadrl_map1_ob5_vel_02.bag")
    newBag("cadrl_03", fn, "b", "bags/scenaries/cadrl/cadrl_map1_ob5_vel_03.bag")

    newBag("arena_01", fn, "g", "bags/scenaries/arena/arena2d_map1_real_ob5_vel_01.bag")

    newBag("dwa_01", fn, "k", "bags/scenaries/dwa/dwa_map1_ob5_vel_01.bag")
    newBag("dwa_02", fn, "k", "bags/scenaries/dwa/dwa_map1_ob5_vel_02.bag")
    newBag("dwa_03", fn, "k", "bags/scenaries/dwa/dwa_map1_ob5_vel_03.bag")

    newBag("teb_01", fn, "r", "bags/scenaries/teb/teb_map1_ob5_vel_01.bag")
    newBag("teb_02", fn, "r", "bags/scenaries/teb/teb_map1_ob5_vel_02.bag")
    newBag("teb_03", fn, "r", "bags/scenaries/teb/teb_map1_ob5_vel_03.bag")

    newBag("mpc_01", fn, "p", "bags/scenaries/mpc/mpc_map1_ob5_vel_01.bag")
    newBag("mpc_02", fn, "p", "bags/scenaries/mpc/mpc_map1_ob5_vel_02.bag")
    newBag("mpc_03", fn, "p", "bags/scenaries/mpc/mpc_map1_ob5_vel_03.bag")

    start_x = 0

    # obstacle_map1_obs10.json
    fn = "obstacle_map1_obs10.txt"
    newBag("cadrl_01", fn, "b", "bags/scenaries/cadrl/cadrl_map1_ob10_vel_01.bag")
    newBag("cadrl_02", fn, "b", "bags/scenaries/cadrl/cadrl_map1_ob10_vel_02.bag")
    newBag("cadrl_03", fn, "b", "bags/scenaries/cadrl/cadrl_map1_ob10_vel_03.bag")

    newBag("arena_01", fn, "g", "bags/scenaries/arena/arena2d_map1_real_ob10_vel_01.bag")

    newBag("dwa_01", fn, "k", "bags/scenaries/dwa/dwa_map1_ob10_vel_01.bag")
    newBag("dwa_02", fn, "k", "bags/scenaries/dwa/dwa_map1_ob10_vel_03.bag")

    newBag("teb_01", fn, "r", "bags/scenaries/teb/teb_map1_ob10_vel_01.bag")
    newBag("teb_02", fn, "r", "bags/scenaries/teb/teb_map1_ob10_vel_02.bag")
    newBag("teb_03", fn, "r", "bags/scenaries/teb/teb_map1_ob10_vel_03.bag")

    newBag("mpc_01", fn, "p", "bags/scenaries/mpc/mpc_map1_ob10_vel_01.bag")
    newBag("mpc_02", fn, "p", "bags/scenaries/mpc/mpc_map1_ob10_vel_02.bag")
    newBag("mpc_03", fn, "p", "bags/scenaries/mpc/mpc_map1_ob10_vel_03.bag")


    # obstacle_map1_obs20.json
    fn = "obstacle_map1_obs20.txt"
    newBag("cadrl_01", fn, "b", "bags/scenaries/cadrl/cadrl_map1_ob20_vel_01.bag")
    newBag("cadrl_02", fn, "b", "bags/scenaries/cadrl/cadrl_map1_ob20_vel_02.bag")
    newBag("cadrl_03", fn, "b", "bags/scenaries/cadrl/cadrl_map1_ob20_vel_03.bag")

    newBag("arena_01", fn, "g", "bags/scenaries/arena/arena2d_map1_real_real_ob20_vel_01.bag")
    newBag("arena_02", fn, "g", "bags/scenaries/arena/arena2d_map1_real_real_ob20_vel_02.bag")
    newBag("arena_03", fn, "g", "bags/scenaries/arena/arena2d_map1_real_real_ob20_vel_03.bag")
    
    newBag("dwa_01", fn, "k", "bags/scenaries/dwa/dwa_map1_ob20_vel_01.bag")
    newBag("dwa_02", fn, "k", "bags/scenaries/dwa/dwa_map1_ob20_vel_02.bag")
    newBag("dwa_03", fn, "k", "bags/scenaries/dwa/dwa_map1_ob20_vel_03.bag")

    newBag("mpc_01", fn, "p", "bags/scenaries/mpc/mpc_map1_ob20_vel_01.bag")
    newBag("mpc_02", fn, "p", "bags/scenaries/mpc/mpc_map1_ob20_vel_02.bag")
    newBag("mpc_03", fn, "p", "bags/scenaries/mpc/mpc_map1_ob20_vel_03.bag")
    
    newBag("teb_01", fn, "r", "bags/scenaries/teb/teb_map1_ob20_vel_01.bag")
    newBag("teb_02", fn, "r", "bags/scenaries/teb/teb_map1_ob20_vel_02.bag")
    newBag("teb_03", fn, "r", "bags/scenaries/teb/teb_map1_ob20_vel_03.bag")

    # map_empty_obs5.json
    fn = "map_empty_obs5.txt"
    newBag("cadrl_01", fn, "b", "bags/scenaries/cadrl/cadrl_map_empty_ob5_vel_01.bag")

    newBag("arena_01", fn, "g", "bags/scenaries/arena/arena2d_empty_ob5_vel_01.bag")

    newBag("dwa_01", fn, "k", "bags/scenaries/dwa/dwa_map_empty_ob5_vel_01.bag")
    newBag("dwa_01", fn, "k", "bags/scenaries/dwa/dwa_map_empty_ob5_vel_02.bag")
    newBag("dwa_01", fn, "k", "bags/scenaries/dwa/dwa_map_empty_ob5_vel_03.bag")

    newBag("mpc_01", fn, "p", "bags/scenaries/mpc/mpc_map_empty_obs5_vel_01.bag")
    newBag("mpc_02", fn, "p", "bags/scenaries/mpc/mpc_map_empty_obs5_vel_02.bag")

    newBag("teb_01", fn, "r", "bags/scenaries/teb/teb_map1_ob5_vel_01.bag")
    newBag("teb_02", fn, "r", "bags/scenaries/teb/teb_map1_ob5_vel_02.bag")
    newBag("teb_03", fn, "r", "bags/scenaries/teb/teb_map1_ob5_vel_03.bag")

    # map_empty_obs10.json
    fn = "_map_empty_obs10.txt"
    newBag("cadrl_01", fn, "b", "bags/scenaries/cadrl/cadrl_map_empty_ob10_vel_01.bag")
    newBag("cadrl_02", fn, "b", "bags/scenaries/cadrl/cadrl_map_empty_ob10_vel_02.bag")
    newBag("cadrl_03", fn, "b", "bags/scenaries/cadrl/cadrl_map_empty_ob10_vel_03.bag")

    newBag("arena_01_p1", fn, "g", "bags/scenaries/arena/arena2d_empty_ob10_vel_01_p1.bag")
    newBag("arena_01_p2", fn, "g", "bags/scenaries/arena/arena2d_empty_ob10_vel_01_p2.bag")

    newBag("dwa_01", fn, "k", "bags/scenaries/dwa/dwa_map_empty_ob10_vel_01.bag")
    newBag("dwa_02", fn, "k", "bags/scenaries/dwa/dwa_map_empty_ob10_vel_02.bag")
    newBag("dwa_03", fn, "k", "bags/scenaries/dwa/dwa_map_empty_ob10_vel_03.bag")

    newBag("mpc_01", fn, "p", "bags/scenaries/mpc/mpc_map_empty_ob10_vel_01.bag")
    newBag("mpc_02", fn, "p", "bags/scenaries/mpc/mpc_map_empty_ob10_vel_02.bag")
    newBag("mpc_03", fn, "p", "bags/scenaries/mpc/mpc_map_empty_ob10_vel_03.bag")

    newBag("teb_01", fn, "r", "bags/scenaries/teb/teb_map1_ob10_vel_01.bag")
    newBag("teb_01", fn, "r", "bags/scenaries/teb/teb_map1_ob10_vel_02.bag")
    newBag("teb_01", fn, "r", "bags/scenaries/teb/teb_map1_ob10_vel_03.bag")

    # map_empty_obs20.json
    fn = "map_empty_obs20.txt"
    newBag("cadrl_01", fn, "b", "bags/scenaries/cadrl/cadrl_map_empty_ob20_vel_01.bag")
    newBag("cadrl_01", fn, "b", "bags/scenaries/cadrl/cadrl_map_empty_ob20_vel_02.bag")
    newBag("cadrl_01", fn, "b", "bags/scenaries/cadrl/cadrl_map_empty_ob20_vel_03.bag")

    newBag("arena_01_p1", fn, "g", "bags/scenaries/arena/arena2d_empty_ob20_vel_01_p1.bag")
    newBag("arena_01_p2", fn, "g", "bags/scenaries/arena/arena2d_empty_ob20_vel_01_p2.bag")
    newBag("arena_01_p3", fn, "g", "bags/scenaries/arena/arena2d_empty_ob20_vel_01_p3.bag")

    newBag("dwa_01", fn, "k", "bags/scenaries/dwa/dwa_map_empty_ob20_vel_01.bag")
    newBag("dwa_01", fn, "k", "bags/scenaries/dwa/dwa_map_empty_ob20_vel_02.bag")
    newBag("dwa_01", fn, "k", "bags/scenaries/dwa/dwa_map_empty_ob20_vel_03.bag")

    newBag("mpc_01", fn, "p", "bags/scenaries/mpc/mpc_map_empty_ob20_vel_01.bag")
    newBag("mpc_03", fn, "p", "bags/scenaries/mpc/mpc_map_empty_ob20_vel_02.bag")

    newBag("teb_01", fn, "r", "bags/scenaries/teb/teb_map1_ob20_vel_01.bag")
    newBag("teb_02", fn, "r", "bags/scenaries/teb/teb_map1_ob20_vel_02.bag")
    newBag("teb_03", fn, "r", "bags/scenaries/teb/teb_map1_ob20_vel_03.bag")











    # static map
    # rospy.init_node("eval", anonymous=False)
    # rospy.Subscriber('/flatland_server/debug/layer/static',MarkerArray, getMap)
    # rospy.spin()
    # plt.show()
 


if __name__=="__main__":
    run()

