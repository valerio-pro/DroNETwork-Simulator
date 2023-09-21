from src.routing_algorithms.BASE_routing import BASE_routing
from src.entities.uav_entities import Drone, Packet, HelloPacket, EstimationPacket, ACKPacket, DataPacket
from src.utilities.utilities import euclidean_distance, config
from math import inf, floor, exp
from numpy.random import random, choice

# State-of-the-art "Fully-Echoed Q-Routing" protocol
class FullyEchoedQLearningRouting(BASE_routing):

    def __init__(self, drone: Drone, simulator):
        BASE_routing.__init__(self, drone=drone, simulator=simulator)
        self.taken_actions: dict = {}  # id event: (old_state, old_action)

        self.exploitation_counter: int = 0 # Number of times exploitation was performed
        self.exploration_counter: int = 0 # Number of times exploration was performed

        # Q_x(D, y) time-span it takes for node "x" to deliver a packet to the depot "D" through neighbor node "y".
        # This time-span is measured in "seconds", not in "time-steps" (1 time-step -> 0.15 seconds).
        # The Q-table is actually a Q-vector since there is only one possible destination (i.e., the depot "D")
        self.q_table: dict = {} # Drone Identifier: Q-value

        # Fully-Echoed Q-Routing Parameters
        self.echo_rate: float = 0.3 # Fixed parameter used to update the dynamic learning rate "eta_2"
        self.T_est: float = 1 # Estimate of the average delivery time to reach the depot
        self.T_max: float = 1 # Estimate of the maximum average delivery time to reach the depot (i.e., biggest "T_est" ever recorded)
        self.eta: float = 0.8 # Fixed learning rate
        self.eta_2: float = (self.T_est/self.T_max) * self.eta * self.echo_rate # Dynamic learning rate (it is updated at each routing step)

        # Simulated-Annealing Parameters
        self.k: int = 0 # Current routing step (i.e., number of actions taken)
        self.k_max: int = floor((config.TS_DURATION * config.SIM_DURATION)/3) # Starting maximum number of exploratory actions
        self.T: float = self.k_max # Simulated-annealing temperature, initialized at its maximum value
        self.f: float = 1 # Parameter used to adjust the temperature w.r.t. changes in network stability. Initialized at 1 so that the temperature is not affected by "self.f" in the first few iterations

        # Parameters for keeping track of the last "H" changes in the Q-values
        self.H: int = 10 # Length of the history of Q-values changes to be considered, the paper uses H=10
        self.last_H_updates: list = [1]*self.H # Q-values changes detected in the last "H" iterations, initially all equal to 1 (to avoid "self.f = 0" in the first iterations)
        self.last_H_index: int = -1 # Keep track of what was the last updated index in the "self.last_H_updates" list (needed for the list update)
             
        # Constant that is multiplied with the transmission time before updating Q-values to have more impact on their change.
        # This is done because the transmission time would be always equal to 1 time-step (i.e., 0.15 seconds)
        self.c: int = 3


    # Override of the "drone_reception()" method declared in "Base_routing" class. 
    # Thanks to this override we don't interfere with the standard functionalities
    # of "drone_reception()" used by every other routing algorithm.
    # We added functionalities to specify what the drone should do when it receives
    # a "DataPacket", "ACKPacket" or "EstimationPacket"
    def drone_reception(self, src_drone: Drone, packet: Packet, current_ts: int):
        """ Handle reception of a packet """

        if isinstance(packet, HelloPacket):
            src_id = packet.src_drone.identifier
            self.hello_messages[src_id] = packet # Add packet to our dictionary

        # Node y receives a DataPacket from node x.
        # Node y must send back to node x (through an ACKPacket) the estimate of the delivery time (t_{y->D})
        # and the time of reception of the DataPacket.
        # Node y also sends back the time of forwarding of the DataPacket and the queue time received. This is
        # done to allow node x to use the queue time and compute the transmission time on the fly i.e.,
        # node x doesn't have to maintain data structures to keep track of queue times and time of transmission 
        # of packets.
        elif isinstance(packet, DataPacket):
            self.no_transmission = True
            self.drone.accept_packets([packet]) # Add packet to the drone buffer

            # Build ack for the reception.
            # Difference between reception time and forwarding time is always 1 time-step
            time_of_data_reception = current_ts
            time_of_data_forwarding = packet.time_of_data_forwarding

            # Extract the queue time of the packet and re-initialize the queue time to 0
            queue_time = packet.queue_time
            packet.queue_time = 0

            # Get the neighbors of node y
            opt_neighbors = self.get_opt_neighbors(current_ts)

            # Check if the neighbors of the drone are in the Q-table
            self.check_new_neighbors(opt_neighbors)

            # Compute node y estimate of delivery time to D, i.e., t_{y->D}
            estimate_time_to_depot_y = self.compute_estimate_time_to_depot([neighbor[1] for neighbor in opt_neighbors])

            # Node y sends the ACKPacket to node x
            ack_packet = ACKPacket(self.drone, src_drone, time_of_data_reception, time_of_data_forwarding, queue_time, estimate_time_to_depot_y, self.simulator, packet, current_ts)
            self.unicast_message(ack_packet, self.drone, src_drone, current_ts)

        # Node x receives the ACKPacket from node y.
        # Node x must update the Q-table and send estimation packets to its neighbors
        elif isinstance(packet, ACKPacket):
            self.drone.remove_packets([packet.acked_packet])
            if self.drone.buffer_length() == 0:
                self.current_n_transmission = 0
                self.drone.move_routing = False

            # Receive node y estimate of delivery time to D
            estimate_time_to_depot_y = packet.estimate_time_to_depot_y

            # Compute transmission time and queue time, the transmission time is multiplied by a constant to have more impact on the Q-value update.
            # The transmission time (without considering "self.c") is always equal to 1 time step i.e., 0.15s 
            transmission_time = (packet.time_of_data_reception - packet.time_of_data_forwarding) * config.TS_DURATION * self.c 
            queue_time = packet.queue_time * config.TS_DURATION

            # Update node x Q-table w.r.t. the estimate received from node y (using the fixed learning rate "eta").
            # We record the change in the Q-value in the history
            self.q_table[src_drone.identifier] += self.eta*(transmission_time + queue_time + estimate_time_to_depot_y - self.q_table[src_drone.identifier]) 
            self.update_changes_history(self.q_table[src_drone.identifier])

            # Get the neighbors of node x
            opt_neighbors = self.get_opt_neighbors(current_ts)

            # Check if the neighbors of the drone are in the Q-table
            self.check_new_neighbors(opt_neighbors)

            # Compute node x estimate of delivery time to D, i.e. t_{x->D}
            estimate_time_to_depot_x = self.compute_estimate_time_to_depot([neighbor[1] for neighbor in opt_neighbors])
            
            # Send t_{x->D} to neighbors so that they can update their Q-table
            est_pck = EstimationPacket(estimate_time_to_depot_x, queue_time, transmission_time, current_ts, self.simulator)
            self.broadcast_message(est_pck, self.drone, [neighbor[1] for neighbor in opt_neighbors], current_ts)

        # Update neighbor Q-table using estimate received from node x (using the dynamic learning rate "eta_2")
        elif isinstance(packet, EstimationPacket):
            transmission_time = packet.transmission_time
            queue_time = packet.queue_time
            estimate_time_to_depot_x = packet.estimate_time_to_depot_x

            if src_drone.identifier not in self.q_table:
                src_hello = self.hello_messages[src_drone.identifier]
                self.q_table[src_drone.identifier] = euclidean_distance(src_hello.cur_pos, self.drone.depot.coords)/src_hello.speed

            # Update node x Q-table w.r.t. the estimate received from node x (using "eta_2" learning rate).
            # Here we don't record the change in the Q-value. We record changes only when a DataPacket is forwarded
            self.q_table[src_drone.identifier] += self.eta_2*(transmission_time + queue_time + estimate_time_to_depot_x - self.q_table[src_drone.identifier]) 

    # The "feedback" function is not used (or useful) for the "Fully-Echoed Q-Routing" protocol
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

            action = self.taken_actions[id_event]

            # remove the entry, the action has received the feedback
            del self.taken_actions[id_event]

    # Choose the next relay among the current neighbors for the input packet 
    def relay_selection(self, opt_neighbors: list, packet: Packet) -> Drone:
        """
        This function returns the best relay to send packets.

        @param packet:
        @param opt_neighbors: a list of tuple (hello_packet, source_drone)
        @return: The best drone to use as relay
        """

        # Update the routing step "self.k"
        self.k += 1

        # Check if the current neighbors of the drone are in the Q-table
        self.check_new_neighbors(opt_neighbors)

        neighbors = [neighbor[1] for neighbor in opt_neighbors]

        # Neighbors that never received the current DataPacket.
        # Computed to enforce a (weak) loop-free property
        trimmed_neighbors = list(set(neighbors).difference(packet.hops))

        # Update the temperature "self.T"
        self.update_temperature()

        # Select an action w.r.t. the current temperature.
        # If there are neighbors that never received the packet then consider those neighbors as potential next relays.
        # If all neighbors already received the packet then consider every possible neighbor.
        # This is done to enforce a (weak) loop-free property
        # (unfortunately the paper does not describe how this property is effectively achieved by the drones, so this is our implementation)
        if trimmed_neighbors:
            action = self.action_selection(trimmed_neighbors)
        else:
            action = self.action_selection(neighbors)
            
        # Store your current action --- you can add some stuff if needed to take a reward later
        self.taken_actions[packet.event_ref.identifier] = (action)
        
        # Update the parameter "self.f"
        self.evaluate_parameter_f()

        # Update the dynamic learning rate "eta_2".
        # The update of "eta_2" is done here since it has to be performed at each routing step
        self.update_dynamic_learning_rate(self.simulator.cur_step)

        return action


################ SUPPORT FUNCTIONS ################
    
    # Function based on simulatead annealing to select the next relay for the packet.
    # It basically consists in an epsilon-greedy where "epsilon" depends on the temperature "self.T".
    # It is a workaround of the action selection policy presented in the paper
    def action_selection(self, neighbors: list) -> Drone:
        r = random()
        # When the temperature is high "epsilon" gets closer to 1
        epsilon = exp(-10/self.T)
        # Exploitation 
        if r <= 1-epsilon:
            self.exploitation_counter += 1
            best = inf
            for neighbor in neighbors:
                if self.q_table[neighbor.identifier] < best:
                    action = neighbor
                    best = self.q_table[neighbor.identifier]
        # Exploration
        else:
            self.exploration_counter += 1
            action = choice(neighbors + [self.drone])
        return action

    # Function that evaluates the parameter "self.f". 
    # "self.f" must be in the [0.5, 10] range
    def evaluate_parameter_f(self):
        self.f = 0
        # Compute the total variation in the Q-values over the last "H" routing steps
        for i in range(len(self.last_H_updates)-1):
            self.f += abs(self.last_H_updates[i+1] - self.last_H_updates[i])
        # Scale down the parameter
        self.f = self.f/self.H
        # Safety checks to make sure that "f" is in the [0.5, 10] range
        if self.f < 0.5:
            self.f = 0.5
        elif self.f > 10:
            self.f = 10.0
    
    # Function used to update the temperature, to be executed before selecting the next action.
    # "self.T" must be in the [1, k_max] range
    def update_temperature(self):
        # If we surpass the starting maximum number of exploratory actions, then set "self.T" to 1.
        # We have to do this because in a real world scenario we can't actually know/control the number of
        # packets that a drone will generate/receive and then route
        if self.k > self.k_max:
            self.T = 1
        else:
            self.T = self.k_max/self.k
        # Update "self.T" with parameter "self.f"
        self.T = self.T * self.f 
        # Safety checks to make sure that the temperature is always in the interval [1, k_max]
        if self.T < 1:
            self.T = 1
        if self.T > self.k_max:
            self.T = self.k_max

    # Function used to update the history of the changes in the Q-values.
    # When a drone updates its Q-table (after forwarding a DataPacket) it immediately registers the new Q-value in its history.
    # This update is done ONLY for DataPackets. The history must be related to DataPackets only. 
    # If a drone updates its Q-table after receiving an "EstimationPacket" then it won't update its history
    def update_changes_history(self, new_q_value: float):
        self.last_H_index = (self.last_H_index + 1) % self.H
        self.last_H_updates[self.last_H_index] = new_q_value

    # Function used to update the dynamic learning rate "eta_2".
    # The delivery time estimates "T_est" and "T_max" must be updated first
    def update_dynamic_learning_rate(self, current_ts: int):
        opt_neighbors = self.get_opt_neighbors(current_ts)
        self.check_new_neighbors(opt_neighbors)
        # In the case of this simulator "T_est" is equal just to the minimum Q-value.
        # The original formula to compute "T_est" does a sum over all possible destinations 
        # but in our case we have just one destination. 
        # The simulator simply puts us in a special case
        self.T_est = self.compute_estimate_time_to_depot([neighbor[1] for neighbor in opt_neighbors])
        self.T_max = max(self.T_max, self.T_est)
        self.eta_2 = (self.T_est/self.T_max) * self.eta * self.echo_rate

    # Check if the neighbors of the drone are not in the Q-table
    def check_new_neighbors(self, opt_neighbors: list):
        for hpk, neighbor in opt_neighbors:
            if neighbor.identifier not in self.q_table:
                self.q_table[neighbor.identifier] = euclidean_distance(hpk.cur_pos, self.drone.depot.coords)/hpk.speed

    # Support function used in the "drone_reception()" method.
    # It computes the minimum Q-value in the drone's Q-table
    def compute_estimate_time_to_depot(self, neighbors: list) -> float:
        estimate_time_to_depot = inf
        for neighbor in neighbors:
            if self.q_table[neighbor.identifier] < estimate_time_to_depot:
                estimate_time_to_depot = self.q_table[neighbor.identifier]
        return estimate_time_to_depot

    # Support function used in the "drone_reception()" method
    def get_opt_neighbors(self, current_ts: int) -> list:
        opt_neighbors = []
        for hpk_id in self.hello_messages:
            hpk: HelloPacket = self.hello_messages[hpk_id]
            # check if packet is too old
            if hpk.time_step_creation < current_ts - config.OLD_HELLO_PACKET:
                continue
            opt_neighbors.append((hpk, hpk.src_drone))
        return opt_neighbors

###################################################