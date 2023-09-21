import src.utilities.utilities as util
from src.routing_algorithms.BASE_routing import BASE_routing

# 1) ADD "NONE" TO ROUTING ALGORITHMS IN "utilities/config.py"
# 2) ADD TO LINE 206 OF FILE "uav_entities.py" ---> not in NONE ---> otherwise "feedback" function is called for "none_routing"
# 3) ADD TO LINE 283 OF FILE "uav_entities.py" ---> not in NONE ---> otherwise "feedback" function is called for "none_routing"
# 4) ADD TO "PLOT_DICT" in "plots/config.py" the entry "NONE": {"hatch": "","markers": "v","linestyle": "-","color": plt.cm.tab10(3),"label": "None","x_ticks_positions": list(range(0, 31, 5))}
# 5) UPDATE ACCORDINGLY THE "plots/plot_data.py" FILE
class NoneRouting(BASE_routing):

    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone, simulator)

    # Drones don't route packets, they always keep them
    def relay_selection(self, opt_neighbors, packet):
        """
        This function returns a random relay for packets.

        @param opt_neighbors: a list of tuples (hello_packet, drone)
        @return: a random drone as relay
        """

        return None
