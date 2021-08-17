import os

import numpy as np
import pandas as pd
import rospkg
from matplotlib import pyplot as plt

agent_name = 'all_in_one_agents_teb_rlca_rule03_policy13'
base_dir = rospkg.RosPack().get_path('arena_local_planner_drl')

# dir_npz = os.path.join(base_dir, 'training_logs', 'train_eval_log', agent_name, 'evaluations.npz')
#
# data_npz = np.load(dir_npz)

# success = data['results']
#
# size = success.size
#
# episodes_mean = np.mean(np.reshape(success, (40, (int) (size / 40))), axis=1)
# print(episodes_mean)
# plt.plot(episodes_mean)
#
#
# smooth_factor = 1
# #plt.plot(np.convolve(success.flatten(), np.ones(smooth_factor)/smooth_factor, mode='valid'))
# plt.show()

dir_csv = os.path.join(base_dir, 'training_logs', 'train_eval_log',
                       agent_name + '.monitor.csv')

df = pd.read_csv(dir_csv, sep=',', skiprows=1)
smooth_factor = 40

fig, (ax1, ax2) = plt.subplots(2, 1)

convolved_r = np.convolve(df['r'], np.ones(smooth_factor) / smooth_factor, mode='valid')
ax1.plot(range(convolved_r.shape[0]), convolved_r)
ax1.set_title('reward evaluation episodes')
ax1.set_xlabel('Evaluation episode')
ax1.set_ylabel('Reward')

# convolved_l = np.convolve(df['l'], np.ones(smooth_factor) / smooth_factor, mode='valid')
# ax2.plot(np.arange(0, convolved_l.shape[0] / 10, 0.1), convolved_l * 0.1)
# ax2.set_title('simulation time per episode')
# ax2.set_xlabel('Evaluation episode')
# ax2.set_ylabel('Time in s')

rewards_numpy = np.array((df['r']))
rewards_per_evaluation = rewards_numpy.reshape((40, (int)(len(df['r']) / 40)))
ax2.plot(range(rewards_per_evaluation.shape[1]), np.mean(rewards_per_evaluation, axis=0))

plt.show()