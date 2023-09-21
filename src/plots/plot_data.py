"""
Implement here your plotting functions
Below you can see a print function example.
You should use it as a reference to implement your own plotting function

IMPORTANT: if you need you can and should use other matplotlib functionalities! Use
            the following example only as a reference

The plot workflow is can be summarized as follows:

    1) Extensive simulations
    2) Json file containing results
    3) Compute averages and stds for each metric for each algorithm
    4) Plot the results

In order to maintain the code tidy you can use:

    - src.plots.config.py file to store all the parameters you need to
        get wonderful plots (see the file for an example)

    - src.plots.data.data_elaboration.py file to write the functions that compute averages and stds from json
        result files

    - src.plots.plot_data.py file to make the plots.

The script plot_data.py can be run using python -m src.plots.plot_data
"""

# THE "plots/config.py" FILE MAY NEED UPDATES
from src.plots.data.data_elaboration import compute_data_avg_std
from src.plots.errorbar_split import errorbar_split
from src.plots.plot_split import plot_split
from src.plots.barplot_split import barplot_split
from src.plots.config import FIG_WIDTH, FIG_HEIGHT 

# RUN WITH "python -m src.plots.plot_data" IN VIRTUAL ENVIRONMENT
if __name__ == "__main__":
    """
    Run this file to get the plots.
    Of course, since you need to plot more than a single data series (one for each algorithm) you need to modify
    plot() in a way that it can handle a multi-dimensional data (one data series for each algorithm). 
    y_data and y_data_std could be for example a list of lists or a dictionary containing lists. It is up to you to decide
    how to deal with data
    """

    # mean_dict = {} -> (n_drones, routing_algorithm): (mean_delivery_time, mean_delivery_ratio, mean_number_relays)
    # std_dict = {} -> (n_drones, routing_algorithm): (std_delivery_time, std_delivery_ratio, std_number_relays)

    algorithms = ["FEQR", "QL", "RND", "GEO"]
    metrics = ["Packet Delivery Time", "Packet Delivery Ratio", "Number of Relays"]
    n_drones = list(range(5, 31, 5))
    _, _, dict_times, dict_relays, dict_ratios = compute_data_avg_std()
    barplot_split(dict_times=dict_times, dict_relays=dict_relays, dict_ratios=dict_ratios, width=FIG_WIDTH, height=FIG_HEIGHT)