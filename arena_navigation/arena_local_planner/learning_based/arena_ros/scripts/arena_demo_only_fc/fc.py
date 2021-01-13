import torch
import torch.nn as nn

HIDDEN_SHAPE_1 = 256
HIDDEN_SHAPE_2 = 256
HIDDEN_SHAPE_3 = 128
HIDDEN_SHAPE_4=  64
# fully connected dqn
class FC_DQN(nn.Module):
	def __init__(self, input_shape, n_actions):
		super(FC_DQN, self).__init__()

		self.sequential = nn.Sequential(nn.Linear(input_shape, HIDDEN_SHAPE_1),
										nn.ReLU(),
										nn.Linear(HIDDEN_SHAPE_1, HIDDEN_SHAPE_2),
										nn.ReLU(),
										nn.Linear(HIDDEN_SHAPE_2, HIDDEN_SHAPE_3),
										nn.ReLU(),
										nn.Linear(HIDDEN_SHAPE_3, HIDDEN_SHAPE_4),
										nn.ReLU(),
										nn.Linear(HIDDEN_SHAPE_4, HIDDEN_SHAPE_4),
										nn.ReLU(),										
										nn.Dropout(),
										nn.Linear(HIDDEN_SHAPE_4, n_actions))

	def forward(self, x):
		return self.sequential(x)
