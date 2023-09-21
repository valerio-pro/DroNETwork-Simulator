"""
Write your plot configuration script here
Below you can see just an example about how to write a config file.
You can import constants, lists and dictionaries in plot_data.py
"""

# *** EXAMPLE ***
import numpy as np
import matplotlib.pyplot as plt

FIG_WIDTH = 8.5
FIG_HEIGHT = 6.5

LABEL_SIZE = 26 # default is 22
LEGEND_SIZE = 22 # default is 20
TITLE_SIZE = 26 # default is 26
TICKS_SIZE = 23 # default is 20
OTHER_SIZES = 21 # default is 20

# The default seaborn color palette is "tab10"
# Try also "deep", "muted", "dark", "pastel", "bright", "colorblind"
# Or even "hls", "husl"
SEABORN_PALETTE = "tab10"

METRICS_OF_INTEREST = [
    "number_of_packets_to_depot",
    "packet_delivery_ratio",
    "packet_mean_delivery_time",
    "mean_number_of_relays"
]

SCALE_LIM_DICT = {
    "number_of_packets_to_depot": {
        "scale": "linear",
        "ylim": (0, 1000)
    },
    "packet_mean_delivery_time": {
        "scale": "linear",
        "ylim": (0, 5)
    },
    "mean_number_of_relays": {
        "scale": "linear",
        "ylim": (0, 10)
    }
}

PLOT_DICT = {
    "QL": {
        "hatch": "",
        "markers": "X",
        "linestyle": "-",
        "color": plt.cm.tab10(0),
        "label": "QL",
        "x_ticks_positions": list(range(0, 31, 5))
    },
    "DBQL": {
        "hatch": "",
        "markers": "h",
        "linestyle": "-",
        "color": plt.cm.tab10(6),
        "label": "DBQL",
        "x_ticks_positions": list(range(0, 31, 5))
    },
    "FEQR": {
        "hatch": "",
        "markers": "v",
        "linestyle": "-",
        "color": plt.cm.tab10(5),
        "label": "FEQR",
        "x_ticks_positions": list(range(0, 31, 5))
    },
    "UCBQL": {
        "hatch": "",
        "markers": "o",
        "linestyle": "-",
        "color": plt.cm.tab10(3),
        "label": "UCB QL",
        "x_ticks_positions": list(range(0, 31, 5))
    },
    "GEO": {
        "hatch": "",
        "markers": "p",
        "linestyle": "-",
        "color": plt.cm.tab10(1),
        "label": "C2S",
        "x_ticks_positions": list(range(0, 31, 5))
    },
    "RND": {
        "hatch": "",
        "markers": "s",
        "linestyle": "-",
        "color": plt.cm.tab10(8),
        "label": "Random",
        "x_ticks_positions": list(range(0, 31, 5))
    },
    "NONE": {
        "hatch": "",
        "markers": "2",
        "linestyle": "-",
        "color": plt.cm.tab10(7),
        "label": "None",
        "x_ticks_positions": list(range(0, 31, 5))
    }
}