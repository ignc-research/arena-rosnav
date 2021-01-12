import gru
from torch.nn.utils.rnn import pack_sequence
import torch
import numpy as np

#parameters
NUM_ACTIONS = 5
num_observations=364
additional_state=2
BATCH_SIZE = 64
USE_GRU = True
SEQ_LENGTH=64
SEQ_LENGTH_MAX=300
DISCOUNT_FACTOR=0.99

#reset()
episode_idx=0
last_action=-1
tensor_state_buffer = torch.zeros(SEQ_LENGTH_MAX, num_observations,dtype=torch.float)
    


# Input
# pack goal position relative to robot
angle = 0      #in rad
distance = 0    #in meter
last_action= -1
last_value=0   # q value

#lidarscan (laser)
sample=np.zeros([360,]).tolist()
obervation_input=[last_value]+[last_action]+[angle]+[distance]+sample



#load NN
model_name="dqn_agent_best_360_gru.dat"
net = gru.GRUModel(num_observations, NUM_ACTIONS)
net.train(False)# set training mode to false to deactivate dropout layer

h = net.init_hidden(1).data
net.load_state_dict(torch.load(model_name, map_location=torch.device('cpu')))

##output NN
prediction=step(obervation_input)

print("action:",prediction)

def step( observation):
	# passing observation through net
	q = None
	tensor_state_buffer[episode_idx] = torch.FloatTensor(observation);
	if episode_idx > SEQ_LENGTH-1:
		start_index= episode_idx-(SEQ_LENGTH-1)
		L=SEQ_LENGTH
	else:
		start_index = 0
		L=episode_idx+1
	state_v=[torch.narrow(tensor_state_buffer, dim=0, start=start_index, length=L)]
	t=pack_sequence(state_v, enforce_sorted=False)
	q,_ = net(t,h)
	q=q.view(-1,NUM_ACTIONS)
	
	# select action with max q value
	_, act_v = torch.max(q, dim=1)
	action = int(act_v.item())
	last_action = action
	return action






