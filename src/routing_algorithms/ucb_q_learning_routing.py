from src.routing_algorithms.BASE_routing import BASE_routing
from src.utilities import utilities as util
from src.entities.uav_entities import Drone, Packet
import math
import numpy as np
import random

'''
RULE: all the action values have to be between 0 and 1
'''

ALPHA: float = 0.5
GAMMA: float = 0.5
C: int = 16

class UCBQLearningRouting(BASE_routing):

    def __init__(self, drone: Drone, simulator):
        BASE_routing.__init__(self, drone=drone, simulator=simulator)
        self.taken_actions = {} # id event : (old_state, old_action)
        self.q_table = {} # state : [action1, action2, action3, ...]
        self.ucb_actions = {}
        
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

            state, chosen_drone, action_step, next_state = self.taken_actions[id_event]
            
            chosen_q_table = chosen_drone.routing_algorithm.q_table

            action = chosen_drone.identifier
            
            successive_cell_idx = util.TraversedCells.coord_to_cell(size_cell=self.simulator.prob_size_cell,
                                                       width_area=self.simulator.env_width,
                                                        x_pos=next_state[0],  
                                                        y_pos=next_state[1])[0]  

            if successive_cell_idx not in chosen_q_table:
                state_optimistic_action_values = np.array([random.random() for drones in range(self.simulator.n_drones)])
                sum_of_values = np.sum(state_optimistic_action_values)
                chosen_q_table[successive_cell_idx] = state_optimistic_action_values/sum_of_values

            # Use the q_learning control algorithm to update the q_table
            q_sa = self.q_table[state][action]
            
            '''
            Observation on the q_learning approach
            When calculating the state_action's q_table_value, we are considering the 
            q_values of his successor which includes every drone in the network, this shouldn't
            concern us since the q_table is exclusive to every drone, but here we are using the q_table of this drone not the q_table
            of his successor
            '''

            # compute reward
            reward = self.reward_function(action_step, delay, outcome)

            best_chosen_action_value = np.max(chosen_q_table[successive_cell_idx])

            # update q_table
            self.q_table[state][action] = q_sa + ALPHA*(reward + GAMMA* best_chosen_action_value - q_sa)

            # remove the entry, the action has received the feedback
            del self.taken_actions[id_event]

    def relay_selection(self, opt_neighbors: list, packet: Packet) -> Drone:
        """
        This function returns the best relay to send packets.

        @param packet:
        @param opt_neighbors: a list of tuple (hello_packet, source_drone)
        @return: The best drone to use as relay
        """

        cell_idx = util.TraversedCells.coord_to_cell(size_cell=self.simulator.prob_size_cell,
                                                       width_area=self.simulator.env_width,
                                                        x_pos=self.drone.coords[0],  
                                                        y_pos=self.drone.coords[1])[0]  
        
        state, chosen_drone = cell_idx, self.drone
        # We need to make sure that there are values in the q_table before updating the estimate of each action
        
        # If there are neighbors
        if opt_neighbors is not []:
            # We need to know the action associated to their state
            if cell_idx not in self.q_table:
                state_optimistic_action_values = np.array([random.random() for drones in range(self.simulator.n_drones)])
                sum_of_values = np.sum(state_optimistic_action_values)
                self.q_table[cell_idx] = state_optimistic_action_values/sum_of_values

            if cell_idx not in self.ucb_actions:
                self.ucb_actions[cell_idx] = [Action(drone, cell_idx) for drone in self.simulator.drones]

            max_estimate = self.ucb_actions[cell_idx][self.drone.identifier].update_estimate(self.simulator.cur_step, 
                self.q_table[cell_idx][self.drone.identifier], C)

            # We must first update each estimate, and to do so we need the current time step and action values
            for neigh_drone in opt_neighbors:
                self.ucb_actions[cell_idx][neigh_drone[1].identifier].update_estimate(self.simulator.cur_step, 
                    self.q_table[cell_idx][neigh_drone[1].identifier], C)

                neigh_action_estimate = self.ucb_actions[cell_idx][neigh_drone[1].identifier].estimate
                if neigh_action_estimate > max_estimate:
                    max_estimate = neigh_action_estimate
                    chosen_drone = neigh_drone[1]
        
        action = self.ucb_actions[cell_idx][chosen_drone.identifier]
        
        # We need to remind it how many times it was selected for future ucb estimates
        action.update_selection_count()

        # Store your current action --- you can add some stuff if needed to take a reward later
        # The step when the action was taken to check 
        self.taken_actions[packet.event_ref.identifier] = (state, chosen_drone, self.simulator.cur_step, chosen_drone.coords)
        return chosen_drone  # here you should return a drone object!

    def reward_function(self, action_step, delay: int, outcome: int):
        # Piu' ti trovi vicino all'esito piu' le tue azioni contano, e piu' le conseguenze sono pesanti (positive/negative reward)
        return (action_step/self.simulator.cur_step)*(self.simulator.event_duration/delay)*outcome
        
# Knows about the drone and the state's cell
# Also keeps track of the estimate for UCB
class Action:

    def __init__(self, drone: Drone, cell_idx: int):
        self.id = drone.identifier
        self.drone = drone
        self.cell_idx = cell_idx
        self.total_selections = 1 # Had to add 1 to avoid division by 0
        self.estimate = 0

    # "cur_step" is the current time step
    # Everytime the estimate is updated is because we selected the action
    def update_estimate(self, cur_step: int, action_value: float, c):
        self.estimate = action_value + math.sqrt(c * math.log(cur_step) / self.total_selections)
        return self.estimate

    def update_selection_count(self):
        self.total_selections += 1

    def __hash__(self) -> int:
        return hash((self.id+self.cell_idx))

    def __eq__(self, other) -> bool:
        if isinstance(other, Action):
            return self.id == other.id and self.cell_idx == other.cell_idx
        return False

