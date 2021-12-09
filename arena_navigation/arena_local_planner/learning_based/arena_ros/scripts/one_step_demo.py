import fc
from torch.nn.utils.rnn import pack_sequence
import torch
import numpy as np

#parameters
NUM_ACTIONS = 5
num_observations=362
SEQ_LENGTH=64
SEQ_LENGTH_MAX=300



def step( observation):
        # passing observation through net
        state_v = torch.FloatTensor([observation]).to('cpu')
        q_vals_v = net(state_v)
        # select action with max q value
        _, act_v = torch.max(q_vals_v, dim=1)
        action = int(act_v.item())
        return action
    


# Input
# pack goal position relative to robot
angle = 0      #in degree
distance = 0    #in meter
##lidarscan


sample=np.ones([360,]).tolist()
observation_input=[angle]+[distance]+sample



#load NN
model_name="dqn_agent_best_fc_l2.dat"
net = fc.FC_DQN(num_observations, NUM_ACTIONS)
net.train(False)# set training mode to false to deactivate dropout layer

net.load_state_dict(torch.load(model_name, map_location=torch.device('cpu')));

##output NN
prediction=step(observation_input)

print("action:",prediction)


