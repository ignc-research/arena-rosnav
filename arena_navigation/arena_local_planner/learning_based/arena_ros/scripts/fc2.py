import torch
import torch.nn as nn

HIDDEN_SHAPE = 64
# fully connected dqn
class FC_DQN(nn.Module):
	def __init__(self, input_shape, n_actions):
		super(FC_DQN, self).__init__()

		self.sequential = nn.Sequential(nn.Linear(input_shape, HIDDEN_SHAPE),
										nn.ReLU(),
										nn.Linear(HIDDEN_SHAPE, HIDDEN_SHAPE),
										nn.ReLU(),
										nn.Dropout(),
										nn.Linear(HIDDEN_SHAPE, n_actions))

	def forward(self, x):
		return self.sequential(x)