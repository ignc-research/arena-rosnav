import os
import matplotlib.pyplot as plt

import rospkg
import numpy as np


agent_name = 'all_in_one_0.5'
base_dir = rospkg.RosPack().get_path('arena_local_planner_drl')

dir = os.path.join(base_dir, 'training_logs', 'train_eval_log', agent_name, 'evaluations.npz')

data = np.load(dir)

success = data['results']

N = 15
plt.plot(np.convolve(success.flatten(), np.ones(N)/N, mode='valid'))
plt.show()
