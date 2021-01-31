from scenario_eval import newBag
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure


def run():
    global ax, start_x
    fig, ax = plt.subplots(figsize=(6, 12))

    start_x = 0.5
    fn = "eval_ob5_v01.txt"
    ns = "_map1_ob5_01"
    newBag("cadrl" + ns, fn, "b", "bags/scenaries/cadrl/cadrl_map1_ob5_vel_01.bag")
    newBag("arena" + ns, fn, "g", "bags/scenaries/arena/arena2d_map1_real_ob5_vel_01.bag")
    newBag("dwa" + ns, fn, "k", "bags/scenaries/dwa/dwa_map1_ob5_vel_03.bag")
    newBag("teb" + ns, fn, "r", "bags/scenaries/teb/teb_map1_ob5_vel_03.bag")
    newBag("mpc" + ns, fn, "p", "bags/scenaries/mpc/mpc_map1_ob5_vel_03.bag")