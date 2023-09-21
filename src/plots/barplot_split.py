import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
from src.plots.config import LABEL_SIZE, LEGEND_SIZE, TICKS_SIZE, SEABORN_PALETTE

def parse_algorithm_name(algorithm: str, long_parse: bool = False) -> str:
    if algorithm == "GEO":
        return "C2S"
    if algorithm == "RND":
        return "Random"
    if algorithm == "FEQR" and long_parse:
        return "Fully-Echoed Q-Routing"
    if algorithm == "QL" and long_parse:
        return "Q-Learning"
    if algorithm == "UCBQL" and long_parse:
        return "UCB Q-Learning"
    if algorithm == "DBQL" and long_parse:
        return "Distance-Based Q-Learning"
    if algorithm == "NONE":
        return "None"
    return algorithm

# If you want ticks also on the right side of the barplots then add "labelright=True" to the arguments of "ax1.tick_params(...)"
def barplot_split(dict_times: dict, dict_relays: dict, dict_ratios: dict, width: float, height: float):
    barplot_packet_delivery_ratio(dict_ratios=dict_ratios, palette=SEABORN_PALETTE, width=width, height=height)
    barplot_packet_delivery_time(dict_times=dict_times, palette=SEABORN_PALETTE, width=width, height=height)
    barplot_number_relays(dict_relays=dict_relays, palette=SEABORN_PALETTE, width=width, height=height)

def barplot_packet_delivery_ratio(dict_ratios: dict, palette: str, width: float, height: float):
    data = {"Number of Drones": [], "Algorithm": [], "Packet Delivery Ratio": []}

    for n, algo in dict_ratios:
        value = dict_ratios[(n, algo)]
        parsed_algorithm = parse_algorithm_name(algorithm=algo, long_parse=False)
        for ratio in value:
            data["Number of Drones"].append(n)
            data["Algorithm"].append(parsed_algorithm)
            data["Packet Delivery Ratio"].append(ratio)

    df = pd.DataFrame(data, columns=["Number of Drones", "Algorithm", "Packet Delivery Ratio"])
    fig, ax1 = plt.subplots(nrows=1, ncols=1, figsize=(width, height)) 

    ax1.set_ylabel(ylabel="Packet Delivery Ratio", fontsize=LABEL_SIZE)
    ax1.set_xlabel(xlabel="Number of Drones", fontsize=LABEL_SIZE)
    ax1.tick_params(axis='both', which='major', labelsize=TICKS_SIZE)
    ax1.set_yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0]) # don't draw the y-tick 1.1
    ax1.set_ylim(bottom=0, top=1.29)

    legend_labels = ["FEQR", "QL", "Random", "C2S"]
    sns.barplot(data=df, x="Number of Drones", y="Packet Delivery Ratio", hue="Algorithm", hue_order=legend_labels, palette=palette, capsize=0.1, ci="sd")
    plt.legend(ncol=2, loc="upper left", handletextpad=0.1, columnspacing=2, prop={'size': LEGEND_SIZE})
    plt.grid(linewidth=0.3)
    plt.tight_layout()
    plt.savefig("src/plots/figures-bars/packet_delivery_ratio.svg")
    plt.savefig("src/plots/figures-bars/packet_delivery_ratio.png", dpi=400)
    plt.clf()

def barplot_packet_delivery_time(dict_times: dict, palette: str, width: float, height: float):
    data = {"Number of Drones": [], "Algorithm": [], "Packet Delivery Time (seconds)": []}

    for n, algo in dict_times:
        value = dict_times[(n, algo)]
        parsed_algorithm = parse_algorithm_name(algorithm=algo, long_parse=False)
        for time in value:
            data["Number of Drones"].append(n)
            data["Algorithm"].append(parsed_algorithm)
            data["Packet Delivery Time (seconds)"].append(time)

    df = pd.DataFrame(data, columns=["Number of Drones", "Algorithm", "Packet Delivery Time (seconds)"])
    fig, ax1 = plt.subplots(nrows=1, ncols=1, figsize=(width, height)) 

    ax1.set_ylabel(ylabel="Packet Delivery Time (seconds)", fontsize=LABEL_SIZE)
    ax1.set_xlabel(xlabel="Number of Drones", fontsize=LABEL_SIZE)
    ax1.tick_params(axis='both', which='major', labelsize=TICKS_SIZE)
    ax1.set_yticks([0, 25, 50, 75, 100, 125])
    ax1.set_ylim(bottom=0, top=174)

    legend_labels = ["FEQR", "QL", "Random", "C2S"]
    sns.barplot(data=df, x="Number of Drones", y="Packet Delivery Time (seconds)", hue="Algorithm", hue_order=legend_labels, palette=palette, capsize=0.1, ci="sd")
    plt.legend(ncol=2, loc="upper left", handletextpad=0.1, columnspacing=2, prop={'size': LEGEND_SIZE})
    plt.grid(linewidth=0.3)
    plt.tight_layout()
    plt.savefig("src/plots/figures-bars/packet_delivery_time.svg")
    plt.savefig("src/plots/figures-bars/packet_delivery_time.png", dpi=400)
    plt.clf()

def barplot_number_relays(dict_relays: dict, palette: str, width: float, height: float):
    data = {"Number of Drones": [], "Algorithm": [], "Number of Relays": []}

    for n, algo in dict_relays:
        value = dict_relays[(n, algo)]
        parsed_algorithm = parse_algorithm_name(algorithm=algo, long_parse=False)
        for num in value:
            data["Number of Drones"].append(n)
            data["Algorithm"].append(parsed_algorithm)
            data["Number of Relays"].append(num)

    df = pd.DataFrame(data, columns=["Number of Drones", "Algorithm", "Number of Relays"])
    fig, ax1 = plt.subplots(nrows=1, ncols=1, figsize=(width, height)) 

    ax1.set_ylabel(ylabel="Number of Relays", fontsize=LABEL_SIZE)
    ax1.set_xlabel(xlabel="Number of Drones", fontsize=LABEL_SIZE)
    ax1.tick_params(axis='both', which='major', labelsize=TICKS_SIZE)
    ax1.set_ylim(bottom=0, top=3)

    legend_labels = ["FEQR", "QL", "Random", "C2S"]
    sns.barplot(data=df, x="Number of Drones", y="Number of Relays", hue="Algorithm", hue_order=legend_labels, palette= palette, capsize=0.1, ci="sd")
    plt.legend(ncol=1, loc="upper left", handletextpad=0.1, columnspacing=0.7, prop={'size': LEGEND_SIZE})
    plt.grid(linewidth=0.3)
    plt.tight_layout()
    plt.savefig("src/plots/figures-bars/number_of_relays.svg")
    plt.savefig("src/plots/figures-bars/number_of_relays.png", dpi=400)
    plt.clf()