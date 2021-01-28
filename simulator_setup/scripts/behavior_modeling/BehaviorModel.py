import numpy as np
import networkx as nx
import constants

class BehaviorModel():

    def __init__(self, recorded_positions, recorded_velocities):
        """Construct BehaviorModel.

        arguments:
        recorded_positions -- Nx2 matrix, N observed positions (x, y)
        recorded_velocities -- N length array containing the recorded velocity at each observation
        """
        self.positions_standing, self.positions_moving = self.get_heatmap(recorded_positions, recorded_velocities)
        # self.movement_graph = self.get_weighted_graph(self.positions_moving)


    def get_heatmap(self, recorded_positions, recorded_velocities):
        """Generate heatmap.

        arguments:
        recorded_positions -- Nx2 matrix, N observed positions (x, y)
        recorded_velocities -- N length array containing the recorded velocity at each observation

        returns:
        positions_standing -- MAP_HEIGHTxMAP_WIDTH matrix, probability for every position on the map where the vehicle stood still (moving positions have probability = 0)
        positions_moving -- MAP_HEIGHTxMAP_WIDTH matrix, probability for every position on the map where the vehicle was moving (standing still positions have probability = 0)
        """

        recorded_positions = recorded_positions * constants.SCALE_FACTOR
        recorded_positions = recorded_positions.astype("int")
        num_observations = recorded_positions.shape[0]
        occurences = np.zeros((constants.MAP_HEIGHT, constants.MAP_WIDTH), dtype=float)
        velocities_sum = np.zeros((constants.MAP_HEIGHT, constants.MAP_WIDTH), dtype=float)
        for i in range(num_observations):
            occurences[recorded_positions[i, 0], recorded_positions[i, 1]] += 1
            velocities_sum[recorded_positions[i, 0], recorded_positions[i, 1]] += recorded_velocities[i]

        # avoid divide by zero
        occurences[occurences <= 0.0] = 1.0;
        mean_velocities = velocities_sum / occurences

        positions_standing = np.copy(occurences)
        positions_standing[mean_velocities < constants.NOT_MOVING_THRESHOLD] = 0  # remove occurences where vehicle was moving
        positions_standing /= positions_standing.sum()  # get probabilities

        positions_moving = np.copy(occurences)
        positions_moving[mean_velocities >= constants.NOT_MOVING_THRESHOLD] = 0  # remove occurences where vehicle was NOT moving
        positions_moving /= positions_moving.sum()  # get probabilities

        return positions_standing, positions_moving


    def get_targets(self, num_targets):
        """Generate target positions for the wanderer.

        arguments:
        num_targets -- number of targets

        returns:
        list of target positions (x, y)
        """

        targets = []
        choices = np.random.choice(constants.MAP_HEIGHT * constants.MAP_WIDTH, size=num_targets, p=self.positions_standing.flatten())
        for choice in choices:
            x = choice % constants.MAP_HEIGHT
            y = choice // constants.MAP_WIDTH
            targets.append((x, y))
        return targets


    def weight_func(self, val1, val2):
        return 1 - (val1 + val2)


    def get_weighted_graph(self, probabilities):
        """Generate weighted graph based on a map of probabilities. Weight of edge between two vertices is (1 - (p(v_1) + p(v_2))).
        Graph will be a grid graph with diagonals connected.
        This function is supposed to be called with self.positions_moving as an argument.

        arguments:
        probabilities -- MAP_HEIGHTxMAP_WIDTH matrix, probability for every position on the map

        returns:
        weighted graph
        """
        G = nx.Graph()
        for i in range(constants.MAP_HEIGHT - 1):
            for j in range(constants.MAP_WIDTH):
                if j > 0:
                    G.add_edge((i, j), (i+1, j-1), weight=self.weight_func(probabilities[i, j], probabilities[i+1, j-1]))
                G.add_edge((i, j), (i+1, j), weight=self.weight_func(probabilities[i, j], probabilities[i+1, j]))
                if j < constants.MAP_WIDTH - 1:
                    G.add_edge((i, j), (i+1, j+1), weight=self.weight_func(probabilities[i, j], probabilities[i+1, j+1]))
                    G.add_edge((i, j), (i, j+1), weight=self.weight_func(probabilities[i, j], probabilities[i, j+1]))
        return G


    def get_path(self, start, end):
        """Generate list of waypoints from start point to end point. Prefer those positions that the vehicle often uses.

        arguments:
        start -- starting coordinates (x, y)
        end -- end coordinates (x, y)

        returns:
        list of coordinates (x, y)
        """
        # TODO nx.shortest_path has a "weight=" kword argument. does it override the weights of the graph?
        return nx.shortest_path(self.movement_graph, source=start, target=end)

