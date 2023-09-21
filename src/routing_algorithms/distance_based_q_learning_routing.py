from src.routing_algorithms.BASE_routing import BASE_routing
from src.entities.uav_entities import Drone, Packet
from src.utilities.utilities import TraversedCells, euclidean_distance
from math import inf, floor
from numpy.random import random, choice
from copy import deepcopy

class DistanceBasedQLearningRouting(BASE_routing):

    def __init__(self, drone: Drone, simulator):
        BASE_routing.__init__(self, drone=drone, simulator=simulator)
        self.taken_actions: dict = {}  # id event : (old_state, old_action)

        self.alpha: float = 0.3 # learning rate
        self.gamma: float = 0.8 # discount factor
        self.epsilon: float = 0.1 # for exploration-exploitation tradeoff
        self.optimistic_initial_values: int = 5

        # an action is simply a drone (if action == None then action == self.drone)
        self.q_table: dict = {} # state: action: float (q_value)
        self.state_actions: dict = {} # state: action: int (number of times that action has been selected in that state)

        self.exploration_counter: int = 0 # number of times a random action was chosen
        self.exploitation_counter: int = 0 # number of times the drone exploited q_values
        self.q_updates: int = 0 # number of times q_table was updated

    def feedback(self, drone: Drone, id_event: int, delay: int, outcome: int):
        """
        Feedback returned when the packet arrives at the depot or
        Expire. This function have to be implemented in RL-based protocols ONLY
        @param drone: The drone that holds the packet
        @param id_event: The Event id
        @param delay: packet delay
        @param outcome: -1 or 1 (read below)
        @return:
        """

        # outcome can be:
        #   -1 if the packet/event expired;
        #   1 if the packets has been delivered to the depot

        if id_event in self.taken_actions:

            # Drone id and Taken actions
            #print(f"\nIdentifier: {self.drone.identifier}, Taken Actions: {self.taken_actions}, Time Step: {self.simulator.cur_step}")
            
            # feedback from the environment
            #print(drone, id_event, delay, outcome)

            state, action, successor = self.taken_actions[id_event]

            # compute reward
            reward = self.reward_function(delay, outcome)

            max_next_action_value = max(list(self.q_table[successor].values()))

            # update q_table
            self.q_table[state][action] = self.q_table[state][action] + self.alpha*(reward + self.gamma*max_next_action_value - self.q_table[state][action])
            self.q_updates += 1
            
            # remove the entry, the action has received the feedback
            del self.taken_actions[id_event]

    def relay_selection(self, opt_neighbors: list, packet: Packet) -> Drone:
        """
        This function returns the best relay to send packets.

        @param packet:
        @param opt_neighbors: a list of tuple (hello_packet, source_drone)
        @return: The best drone to use as relay
        """
        
        # compute the state the drone is in, if it is a new state then add it to the q_table
        state = State(self.drone, self.simulator)
        if state not in self.q_table:
            self.q_table[state] = {drone: self.optimistic_initial_values for drone in self.simulator.drones}

        # give drones a fair chance to explore some actions before exploiting them
        if self.exploration_counter <= self.simulator.n_drones:
            action = self.random_policy([neighbor[1] for neighbor in opt_neighbors])
        else:
            action = self.distance_based_epsilon_greedy(state, [neighbor[1] for neighbor in opt_neighbors])

        # compute successor state, if it is a new state then add it to the q_table
        successor = state.successor_estimate(self.drone, self.simulator)
        if successor not in self.q_table:
            self.q_table[successor] = {drone: self.optimistic_initial_values for drone in self.simulator.drones}
        if successor not in self.state_actions:
            self.state_actions[successor] = {}

        if action == None:
            action = self.drone

        # Store your current action --- you can add some stuff if needed to take a reward later
        self.taken_actions[packet.event_ref.identifier] = (state, action, successor)
        
        # record actions taken in each state and number of times those actions have been taken in that state
        if state not in self.state_actions:
            self.state_actions[state] = {action: 1}
        else:
            if action in self.state_actions[state]:
                self.state_actions[state][action] += 1
            else:
                self.state_actions[state][action] = 1

        return action  # here you should return a drone object!

    # exploit w.p. 1-epsilon; explore w.p. epsilon
    def distance_based_epsilon_greedy(self, state, neighbors: list) -> Drone:
        if len(neighbors) == 0 or euclidean_distance(self.drone.coords, self.drone.depot.coords) <= self.drone.communication_range:
            return None
        p = random()
        if p <= 1 - self.epsilon:
            return self.distance_greedy_policy(state, neighbors)
        return self.random_policy(neighbors)

    # greedy policy but with weights on the q_values. Weights are given to the q_value of a neighbor considering the distance of that neighbor from the depot
    # neighbors that don't have the maximum q_value but are closer to the depot have a chance to be selected
    # executed w.p. 1-epsilon for action selection
    def distance_greedy_policy(self, state, neighbors: list) -> Drone:
        max = -inf
        best_action = None
        for neighbor in neighbors:
            neighbor_distance_to_depot = euclidean_distance(neighbor.coords, self.drone.depot.coords)
            # i think we never go in the body of this if condition
            if neighbor_distance_to_depot <= neighbor.communication_range:  # communication_range is 200
                return neighbor
            scaled_down_neighbor_distance_to_depot = neighbor_distance_to_depot/100   # we have always something > 1 since communication_range is always 200
            # give weights to q_values with respect to the distance of the neighbor to the depot
            if (1/scaled_down_neighbor_distance_to_depot)*self.q_table[state][neighbor] > max:
                max = self.q_table[state][neighbor]*(1/scaled_down_neighbor_distance_to_depot)
                best_action = neighbor
        self.exploitation_counter += 1
        return best_action

    # random policy for exploration
    # executed w.p. epsilon for action selection
    def random_policy(self, neighbors: list) -> Drone:
        self.exploration_counter += 1
        return choice(neighbors + [None])

    # simple reward function
    def reward_function(self, delay: int, outcome: int) -> int:
        # packet expired -> bad reward
        if outcome == -1:
            return -2
        else:
            # packet delivered -> good reward
            # if the packet is delivered within the first half of its lifetime then we have a little bonus of +1 to the reward for the drone
            return 2 + floor(1000/delay)
    
class State:

    # the state of the drone is given just by the position of the drone itself in the (discretized) AoI
    def __init__(self, drone: Drone, simulator):
        self.id = drone.identifier
        self.cell = TraversedCells.coord_to_cell(size_cell=simulator.prob_size_cell,
            width_area=simulator.env_width,
            x_pos=drone.coords[0],  
            y_pos=drone.coords[1])[0]

    # the successor state is computed as the next position of the drone in the (discretized) AoI
    # this next position is the next waypoint on the drones's path
    # this function is called in "relay_selection()" after a packet is sent off to a neighbor
    # notice that the successor state is not defined by the action taken by the drone
    def successor_estimate(self, drone: Drone, simulator):
        succ = self.copy()
        succ.cell = TraversedCells.coord_to_cell(size_cell=simulator.prob_size_cell,
            width_area=simulator.env_width,
            x_pos=drone.next_target()[0],  
            y_pos=drone.next_target()[1])[0]
        return succ

    def __str__(self) -> str:
        return f'''Drone {self.id} is in cell {self.cell}'''

    def __hash__(self) -> int:
        return hash((self.id))

    def __eq__(self, other) -> bool:
        if isinstance(other, State):
            return self.id == other.id and self.cell == other.cell
        return False

    def copy(self):
        return deepcopy(self)