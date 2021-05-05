import torch
import torch.nn as nn
import numpy as np
import itertools
import logging

#building MLP
def mlp(input_dim, mlp_dims, last_relu=False):
    layers = []
    mlp_dims = [input_dim] + mlp_dims
    for i in range(len(mlp_dims) - 1):
        layers.append(nn.Linear(mlp_dims[i], mlp_dims[i + 1]))
        if i != len(mlp_dims) - 2 or last_relu:
            layers.append(nn.ReLU())
    net = nn.Sequential(*layers)
    return net
#torch implementation of numpy.isin method
def isin(ar1, ar2):
    mask = ar2.new_zeros((max(ar1.max(), ar2.max()) + 1,), dtype=torch.bool)
    mask[ar2.unique()] = True
    return mask[ar1]

class ValueNetwork(nn.Module):
    def __init__(self, input_dim, self_state_dim, mlp1_dims, mlp2_dims, mlp3_dims, attention_dims, with_global_state,
                 cell_size, cell_num,om_channel_size,with_om):
        super().__init__()
        self.self_state_dim = self_state_dim
        self.global_state_dim = mlp1_dims[-1]
        self.mlp1 = mlp(input_dim, mlp1_dims, last_relu=True)
        self.mlp2 = mlp(mlp1_dims[-1], mlp2_dims, last_relu=True)
        self.with_global_state = with_global_state
        self.device='cuda'
        if with_global_state:
            self.attention = mlp(mlp1_dims[-1] * 2, attention_dims)
        else:
            self.attention = mlp(mlp1_dims[-1], attention_dims)
        self.cell_size = cell_size
        self.cell_num = cell_num
        self.with_om = with_om
        self.om_channel_size = om_channel_size
        self.mlp3_input_dim = mlp2_dims[-1] + self.self_state_dim
        self.mlp3 = mlp(self.mlp3_input_dim, mlp3_dims)
        self.attention_weights = None

    def forward(self, state):
        """
        First transform the world coordinates to self-centric coordinates and then do forward computation

        :param state: tensor of shape ( batch_size, # of humans, length of a rotated state )
        :return:
        """
        # size = state.shape
        self_state = state[:, 0, :self.self_state_dim]
        human_states = state[:, :, self.self_state_dim + 2: self.self_state_dim + 6]

        if self.with_om:
            occupancy_maps = build_occupancy_maps(human_states, self.cell_size, self.cell_num, self.om_channel_size)
            state = torch.cat([state, occupancy_maps.to(self.device)], dim=2).float()
        size = state.shape
        # print('size', size)
        mlp1_output = self.mlp1(state) #.view((-1, size[2]))
        mlp2_output = self.mlp2(mlp1_output)

        if self.with_global_state:
            # compute attention scores
            global_state = torch.mean(mlp1_output.view(size[0], size[1], -1), 1, keepdim=True)
            global_state = global_state.expand((size[0], size[1], self.global_state_dim)).view(size[0],-1, self.global_state_dim) #contiguous().
            attention_input = torch.cat([mlp1_output, global_state], dim=2)
        else:
            attention_input = mlp1_output
        
        scores = self.attention(attention_input) .view(size[0], size[1], 1).squeeze(dim=2)

        # masked softmax
        # weights = softmax(scores, dim=1).unsqueeze(2)
        scores_exp = torch.exp(scores) * (scores != 0).float()
        weights = (scores_exp / torch.sum(scores_exp, dim=1, keepdim=True)).unsqueeze(2)
        self.attention_weights = weights[0, :, 0].data.cpu().numpy()
        # print(self.attention_weights)

        # output feature is a linear combination of input features
        features = mlp2_output.view(size[0], size[1], -1)
        # for converting to onnx
        # expanded_weights = torch.cat([torch.zeros(weights.size()).copy_(weights) for _ in range(50)], dim=2)
        weighted_feature = torch.sum(torch.mul(weights, features), dim=1)
        # concatenate agent's state with global weighted humans' state
        joint_state = torch.cat([self_state, weighted_feature], dim=1)
        # print('joint size', joint_state.shape)
        # value = self.mlp3(joint_state)
        return joint_state


def build_occupancy_maps(human_states, cell_size, cell_num, om_channel_size):
    """
    :param human_states:
    :return: tensor of shape (# human - 1, self.cell_num ** 2)
    """
    occupancy_maps = None
    size=human_states.shape
    for i in torch.arange(size[1]):
        human=human_states[:,i,:]
        other_humans = torch.cat([human_states[:, :i, :],human_states[:, i+1:, :]],1)
        other_px = other_humans[:, :, 0] - human[:, 0].reshape(-1,1)
        other_py = other_humans[:, :, 1] - human[:, 1].reshape(-1,1)
        # new x-axis is in the direction of human's velocity
        human_velocity_angle = torch.atan2(human[:,3], human[:,2]).reshape(-1,1)
        other_human_orientation = torch.atan2(other_py, other_px)
        
        rotation = other_human_orientation - human_velocity_angle
        distance = (other_px**2-other_py**2)**.5
        other_px = torch.cos(rotation) * distance
        other_py = torch.sin(rotation) * distance

        # compute indices of humans in the grid
        other_x_index = torch.floor(other_px / cell_size + cell_num / 2)
        other_y_index = torch.floor(other_py / cell_size + cell_num / 2)
        other_x_index[other_x_index < 0] = float('-inf')
        other_x_index[other_x_index >= cell_num] = float('-inf')
        other_y_index[other_y_index < 0] = float('-inf')
        other_y_index[other_y_index >= cell_num] = float('-inf')
        grid_indices = (cell_num * other_y_index + other_x_index).type(torch.LongTensor)
        # occupancy_map = isin(grid_indices,torch.LongTensor(range(cell_num ** 2)))
        occupancy_map_batch=None
        if om_channel_size == 1:
            occupancy_maps.append([occupancy_map.type(torch.int)])
        else:
            # calculate relative velocity for other agents
            other_human_velocity_angles = torch.atan2(other_humans[:,:,3], other_humans[:,:,2])
            rotation = other_human_velocity_angles - human_velocity_angle
            speed = torch.linalg.norm(other_humans[:, :, 2:4],dim=2)

            other_vx = torch.cos(rotation) * speed
            other_vy = torch.sin(rotation) * speed
       
            for k,grid_indice in enumerate(grid_indices):
                dm = [list() for _ in range(cell_num ** 2 * om_channel_size)]
                for i, index in np.ndenumerate(grid_indice):
                    if index in range(cell_num ** 2):
                        if om_channel_size == 2:
                            dm[2 * int(index)].append(other_vx[k,i].item())
                            dm[2 * int(index) + 1].append(other_vy[k,i].item())
                        elif om_channel_size == 3:
                            dm[3 * int(index)].append(1)
                            dm[3 * int(index) + 1].append(other_vx[k,i].item())
                            dm[3 * int(index) + 2].append(other_vy[k,i].item())
                        else:
                            print('Not Implemented Error')
                for i, cell in enumerate(dm):
                    dm[i] = np.sum(dm[i])/ len(dm[i]) if len(dm[i]) != 0 else 0              
                dm=torch.tensor(dm).reshape(1,1,-1)
                if occupancy_map_batch==None:
                    occupancy_map_batch=dm
                else:
                    occupancy_map_batch=torch.cat((occupancy_map_batch,dm),0)
        if occupancy_maps==None:
          occupancy_maps=occupancy_map_batch
        else:
          occupancy_maps=torch.cat((occupancy_maps,occupancy_map_batch),1)
    return occupancy_maps

#[sarl] configs
mlp1_dims = [128, 96]
mlp2_dims = [96, 64]
attention_dims = [96, 96, 1]
mlp3_dims = [150, 100, 100, 1]
multiagent_training = False
with_om = False
with_global_state = True
#[om] configs
cell_num = 24
cell_size = 0.3   # the size of humans
om_channel_size = 3

class SARL(object):
    def __init__(self):
        """
        SARL class net structure used for the according policy.
        """
        self.trainable = True
        self.phase = None
        self.model = None
        self.with_om = with_om
        self.device = 'cuda'
        self.cell_size = cell_size
        self.cell_num = cell_num
        self.om_channel_size = om_channel_size
        self.self_state_dim = 9
        self.human_state_dim = 10
        self.input_dim = self.self_state_dim + self.human_state_dim + (self.cell_num ** 2 * self.om_channel_size if self.with_om else 0)

    def build_net(self):
        self.model = ValueNetwork(self.input_dim, self.self_state_dim, mlp1_dims, mlp2_dims, mlp3_dims,
                                attention_dims, with_global_state, self.cell_size, self.cell_num, self.om_channel_size, self.with_om)
        self.multiagent_training = multiagent_training
        if self.with_om:
            self.name = 'OM-SARL'
        # logging.info('Policy: {} {} global state'.format(self.name, 'w/' if with_global_state else 'w/o'))

    def get_attention_weights(self):
        return self.model.attention_weights
    
    def set_device(self, device):
        self.device = device
        self.model.to(self.device)

# nets for lstm_rl
class ValueNetwork1(nn.Module):
    def __init__(self, input_dim, self_state_dim, mlp_dims, lstm_hidden_dim):
        super().__init__()
        self.self_state_dim = self_state_dim
        self.lstm_hidden_dim = lstm_hidden_dim
        self.mlp = mlp(self_state_dim + lstm_hidden_dim, mlp_dims)
        self.lstm = nn.LSTM(input_dim, lstm_hidden_dim, batch_first=True)

    def forward(self, state):
        """
        First transform the world coordinates to self-centric coordinates and then do forward computation

        :param state: tensor of shape (batch_size, # of humans, length of a joint state)
        :return:
        """
        size = state.shape
        self_state = state[:, 0, :self.self_state_dim]
        # human_state = state[:, :, self.self_state_dim:]
        h0 = torch.zeros(1, size[0], self.lstm_hidden_dim)
        c0 = torch.zeros(1, size[0], self.lstm_hidden_dim)
        output, (hn, cn) = self.lstm(state, (h0, c0))
        hn = hn.squeeze(0)
        joint_state = torch.cat([self_state, hn], dim=1)
        value = self.mlp(joint_state)
        return value


class ValueNetwork2(nn.Module):
    def __init__(self, input_dim, self_state_dim, mlp1_dims, mlp_dims, lstm_hidden_dim):
        super().__init__()
        self.self_state_dim = self_state_dim
        self.lstm_hidden_dim = lstm_hidden_dim
        self.mlp1 = mlp(input_dim, mlp1_dims)
        self.mlp = mlp(self_state_dim + lstm_hidden_dim, mlp_dims)
        self.lstm = nn.LSTM(mlp1_dims[-1], lstm_hidden_dim, batch_first=True)

    def forward(self, state):
        """
        First transform the world coordinates to self-centric coordinates and then do forward computation

        :param state: tensor of shape (batch_size, # of humans, length of a joint state)
        :return:
        """
        size = state.shape
        self_state = state[:, 0, :self.self_state_dim]

        state = torch.reshape(state, (-1, size[2]))
        mlp1_output = self.mlp1(state)
        mlp1_output = torch.reshape(mlp1_output, (size[0], size[1], -1))

        h0 = torch.zeros(1, size[0], self.lstm_hidden_dim)
        c0 = torch.zeros(1, size[0], self.lstm_hidden_dim)
        output, (hn, cn) = self.lstm(mlp1_output, (h0, c0))
        hn = hn.squeeze(0)
        joint_state = torch.cat([self_state, hn], dim=1)
        value = self.mlp(joint_state)
        return value


#need to be modified
def predict(state):
    """
    A base class for all methods that takes pairwise joint state as input to value network.
    The input to the value network is always of shape (batch_size, # humans, rotated joint state length)

    """
    if self.phase is None or self.device is None:
        raise AttributeError('Phase, device attributes have to be set!')
    if self.phase == 'train' and self.epsilon is None:
        raise AttributeError('Epsilon attribute has to be set in training phase')

    if self.reach_destination(state):
        return ActionXY(0, 0) if self.kinematics == 'holonomic' else ActionRot(0, 0)
    if self.action_space is None:
        self.build_action_space(state.self_state.v_pref)

    occupancy_maps = None
    probability = np.random.random()
    if self.phase == 'train' and probability < self.epsilon:
        max_action = self.action_space[np.random.choice(len(self.action_space))]
    else:
        self.action_values = list()
        max_value = float('-inf')
        max_action = None
        for action in self.action_space:
            next_self_state = self.propagate(state.self_state, action)
            if self.query_env:
                next_human_states, reward, done, info = self.env.onestep_lookahead(action)
            else:
                next_human_states = [self.propagate(human_state, ActionXY(human_state.vx, human_state.vy))
                                    for human_state in state.human_states]
                reward = self.compute_reward(next_self_state, next_human_states)
            batch_next_states = torch.cat([torch.Tensor([next_self_state + next_human_state]).to(self.device)
                                            for next_human_state in next_human_states], dim=0)
            rotated_batch_input = self.rotate(batch_next_states).unsqueeze(0)
            if self.with_om:
                if occupancy_maps is None:
                    occupancy_maps = self.build_occupancy_maps(next_human_states).unsqueeze(0)
                rotated_batch_input = torch.cat([rotated_batch_input, occupancy_maps.to(self.device)], dim=2)
            # VALUE UPDATE
            next_state_value = self.model(rotated_batch_input).data.item()
            value = reward + pow(self.gamma, self.time_step * state.self_state.v_pref) * next_state_value
            self.action_values.append(value)
            if value > max_value:
                max_value = value
                max_action = action
        if max_action is None:
            raise ValueError('Value network is not well trained. ')

    if self.phase == 'train':
        self.last_state = self.transform(state)

    return max_action
