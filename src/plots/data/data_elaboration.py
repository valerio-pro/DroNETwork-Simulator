from numpy import mean, std
import json
import os

"""
You can write here the data elaboration function/s

You should read all the JSON files containing simulations results and compute
average and std of all the metrics of interest.

You can find the JSON file from the simulations into the data evaluation_tests folder.
Each JSON file follows the naming convention: simulation-current date-simulation id__seed_drones number_routing algorithm

In this way you can parse the name and properly aggregate the data.

To aggregate data you can use also external libraries such as Pandas!

IMPORTANT: Both averages and stds must be computed over different seeds for the same metric!
"""

def read_json_directory(path: str) -> dict:
    # The argument "path" is the relative path of a directory containing only json files of simulations
    data = {} # (n_drones, routing_algorithm, seed): (packet_mean_delivery_time, packet_delivery_ratio)
    for filename in os.listdir(path):
        if os.path.isfile(os.path.join(path, filename)) and filename != '.gitkeep':
            json_file = os.path.join(path, filename)
            with open(json_file, 'r') as text:
                while text:
                    line = text.readline()
                    if line:
                        json_data = json.loads(line)
                        n_drones = json_data["mission_setup"]["n_drones"]
                        routing_algorithm = json_data["mission_setup"]["routing_algorithm"].split(".")[1]
                        seed = json_data["mission_setup"]["seed"]
                        packet_mean_delivery_time = json_data["packet_mean_delivery_time"]
                        packet_delivery_ratio = json_data["packet_delivery_ratio"]
                        mean_number_of_relays = json_data["mean_number_of_relays"]
                        data[(n_drones, routing_algorithm, seed)] = (packet_mean_delivery_time, packet_delivery_ratio, mean_number_of_relays)
                        #print(data)
                    else:
                        break
    return data
            
def compute_data_avg_std() -> tuple:
    """
    Computes averages and stds from JSON files
    @param path: results folder path
    @return: one or more data structure containing data
    """

    # Get data over all seeds
    mean_dict = {} # (n_drones, routing_algorithm): (mean_delivery_time, mean_delivery_ratio, mean_number_relays)
    std_dict = {} # (n_drones, routing_algorithm): (std_delivery_time, std_delivery_ratio, std_number_relays)

    # Fix number of drones and a routing algorithm and aggregate all the data of a metric (e.g., packet_delivery_ratio) in a list associated to
    # that number of drones and algoirthm. After a simulation campaign the size of each list should be 30, one entry for each seed.
    dict_times = {} # (n_drones, routing_algorithm): list [delivery_times]
    dict_ratios = {} # (n_drones, routing_algorithm): list [delivery_ratios]
    dict_relays = {} # (n_drones, routing_algorithm): list [number_relays]

    path = os.path.join("data", "evaluation_tests")
    data = read_json_directory(path=path)

    for n_drones, routing_algorithm, seed in data:
        pkt_time, pkt_ratio, num_relays = data[(n_drones, routing_algorithm, seed)]
        if pkt_ratio > 1.0:
            pkt_ratio = 1.0
        if (n_drones, routing_algorithm) not in dict_times:
            dict_times[(n_drones, routing_algorithm)] = [pkt_time]
            dict_ratios[(n_drones, routing_algorithm)] = [pkt_ratio]
            dict_relays[(n_drones, routing_algorithm)] = [num_relays]
        else:
            dict_times[(n_drones, routing_algorithm)].append(pkt_time)
            dict_ratios[(n_drones, routing_algorithm)].append(pkt_ratio)
            dict_relays[(n_drones, routing_algorithm)].append(num_relays)

    #print(dict_times)
    #print(dict_ratios)
    #print(dict_relays)

    for n_drones, routing_algorithm in dict_times:
        list_times = dict_times[(n_drones, routing_algorithm)]
        list_ratios = dict_ratios[(n_drones, routing_algorithm)]
        list_relays = dict_relays[(n_drones, routing_algorithm)]
        mean_dict[(n_drones, routing_algorithm)] = (mean(list_times), mean(list_ratios), mean(list_relays))
        std_dict[(n_drones, routing_algorithm)] = (std(list_times), std(list_ratios), std(list_relays))

    #print(mean_dict)
    #print(std_dict)

    return mean_dict, std_dict, dict_times, dict_relays, dict_ratios

if __name__ == "__main__":
    """
    You can run this file to test your script
    """
    #compute_data_avg_std()