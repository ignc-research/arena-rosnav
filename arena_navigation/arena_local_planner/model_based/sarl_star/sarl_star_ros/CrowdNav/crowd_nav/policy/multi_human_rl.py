# Author: Changan Chen <changanvr@gmail.com>
# Modified by: Keyu Li <kyli@link.cuhk.edu.hk>

from __future__ import division
import torch
import numpy as np
from crowd_sim.envs.utils.action import ActionRot, ActionXY
from crowd_nav.policy.cadrl import CADRL


class MultiHumanRL(CADRL):
    def __init__(self):
        super(MultiHumanRL, self).__init__()
        self.with_costmap = False
        self.gc = None
        self.gc_resolution = None
        self.gc_width = None
        self.gc_ox = None
        self.gc_oy = None

    # predict the cost that robot hits the static obstacles in the global map
    def compute_cost(self, state):
        costs = []
        x = state.px
        y = state.py
        min_x = x - 0.3
        min_y = y - 0.3
        max_x = x + 0.3
        max_y = y + 0.3
        grid_min_x = int(round((min_x - self.gc_ox) / self.gc_resolution))
        grid_min_y = int(round((min_y - self.gc_oy) / self.gc_resolution))
        grid_max_x = int(round((max_x - self.gc_ox) / self.gc_resolution))
        grid_max_y = int(round((max_y - self.gc_oy) / self.gc_resolution))
        for i in range(grid_min_x, grid_max_x+1):
            for j in range(grid_min_y, grid_max_y + 1):
                index = i + self.gc_width * j
                costs.append(self.gc[index])
        max_cost = max(costs)
        return max_cost


    def predict(self, state):
        """
        Takes pairwise joint state as input to value network and output action.
        The input to the value network is always of shape (batch_size, # humans, rotated joint state length).
        If with_costmap is True, the dangerous actions predicted by the value network will be screened out to avoid static obstacles on the map.
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
                next_self_state_further = self.propagate_more(state.self_state, action)

                # abort actions which will probably cause collision with static obstacles in the costmap
                cost = self.compute_cost(next_self_state_further)
                if cost > 0:
                    print("********** Abort action:", action, "cost:", cost, "that will hit the obstacles.")
                    continue

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
                    rotated_batch_input = torch.cat([rotated_batch_input, occupancy_maps], dim=2)
                # VALUE UPDATE
                next_state_value = self.model(rotated_batch_input).data.item()
                value = reward + pow(self.gamma, self.time_step * state.self_state.v_pref) * next_state_value
                self.action_values.append(value)
                if value > max_value:
                    max_value = value
                    max_action = action
                    # print("********** choose action:", action)
                    # print("********** cost:", cost)

            if max_action is None:
                # if the robot is trapped, choose the turning action to escape
                max_action = ActionRot(0, 0.78)
                print("The robot is trapped. Rotate in place to escape......")

        if self.phase == 'train':
            self.last_state = self.transform(state)

        return max_action

    def compute_reward(self, nav, humans):
        # collision detection
        dmin = float('inf')
        collision = False
        if len(humans):
            for i, human in enumerate(humans):
                dist = np.linalg.norm((nav.px - human.px, nav.py - human.py)) - nav.radius - human.radius
                if dist < 0:
                    collision = True
                    break
                if dist < dmin:
                    dmin = dist
        # check if reaching the goal
        reaching_goal = np.linalg.norm((nav.px - nav.gx, nav.py - nav.gy)) < nav.radius
        if collision:
            reward = self.env.collision_penalty
        elif reaching_goal:
            reward = 1
        elif dmin < self.env.discomfort_dist:
            reward = (dmin - self.env.discomfort_dist) * self.env.discomfort_penalty_factor * self.env.time_step
        else:
            reward = 0

        return reward

    def transform(self, state):
        """
        Take the state passed from agent and transform it to the input of value network

        :param state:
        :return: tensor of shape (# of humans, len(state))
        """
        state_tensor = torch.cat([torch.Tensor([state.self_state + human_state]).to(self.device)
                                  for human_state in state.human_states], dim=0)
        if self.with_om:
            occupancy_maps = self.build_occupancy_maps(state.human_states)
            state_tensor = torch.cat([self.rotate(state_tensor), occupancy_maps], dim=1)
        else:
            state_tensor = self.rotate(state_tensor)
        return state_tensor

    def input_dim(self):
        return self.joint_state_dim + (self.cell_num ** 2 * self.om_channel_size if self.with_om else 0)
                                                  # a**b means a^b
        # if not with_om, input_dim = joint_state_dim

    def build_occupancy_maps(self, human_states):
        """

        :param human_states:
        :return: tensor of shape (# human - 1, self.cell_num ** 2)
        """
        occupancy_maps = []
        for human in human_states:
            other_humans = np.concatenate([np.array([(other_human.px, other_human.py, other_human.vx, other_human.vy)])
                                         for other_human in human_states if other_human != human], axis=0)
            other_px = other_humans[:, 0] - human.px
            other_py = other_humans[:, 1] - human.py
            # new x-axis is in the direction of human's velocity
            human_velocity_angle = np.arctan2(human.vy, human.vx)
            other_human_orientation = np.arctan2(other_py, other_px)
            rotation = other_human_orientation - human_velocity_angle
            distance = np.linalg.norm([other_px, other_py], axis=0)
            other_px = np.cos(rotation) * distance
            other_py = np.sin(rotation) * distance

            # compute indices of humans in the grid
            other_x_index = np.floor(other_px / self.cell_size + self.cell_num / 2)
            other_y_index = np.floor(other_py / self.cell_size + self.cell_num / 2)
            other_x_index[other_x_index < 0] = float('-inf')
            other_x_index[other_x_index >= self.cell_num] = float('-inf')
            other_y_index[other_y_index < 0] = float('-inf')
            other_y_index[other_y_index >= self.cell_num] = float('-inf')
            grid_indices = self.cell_num * other_y_index + other_x_index
            occupancy_map = np.isin(range(self.cell_num ** 2), grid_indices)
            if self.om_channel_size == 1:
                occupancy_maps.append([occupancy_map.astype(int)])
            else:
                # calculate relative velocity for other agents
                other_human_velocity_angles = np.arctan2(other_humans[:, 3], other_humans[:, 2])
                rotation = other_human_velocity_angles - human_velocity_angle
                speed = np.linalg.norm(other_humans[:, 2:4], axis=1)
                other_vx = np.cos(rotation) * speed
                other_vy = np.sin(rotation) * speed
                dm = [list() for _ in range(self.cell_num ** 2 * self.om_channel_size)]
                for i, index in np.ndenumerate(grid_indices):
                    if index in range(self.cell_num ** 2):
                        if self.om_channel_size == 2:
                            dm[2 * int(index)].append(other_vx[i])
                            dm[2 * int(index) + 1].append(other_vy[i])
                        elif self.om_channel_size == 3:
                            dm[2 * int(index)].append(1)
                            dm[2 * int(index) + 1].append(other_vx[i])
                            dm[2 * int(index) + 2].append(other_vy[i])
                        else:
                            raise NotImplementedError
                for i, cell in enumerate(dm):
                    dm[i] = sum(dm[i]) / len(dm[i]) if len(dm[i]) != 0 else 0
                occupancy_maps.append([dm])

        return torch.from_numpy(np.concatenate(occupancy_maps, axis=0)).float()

