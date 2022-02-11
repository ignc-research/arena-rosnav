import argparse  # get user input from terminal as arguments
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml


def parsing():  # define what flags user can/must give as terminal input
    parser = argparse.ArgumentParser(description='Evaluate ods files.')  # create parser object
    parser.add_argument('path_to_parent_directory', type=str, action='store',
                        help='path to parent directory of directory with the .ods files')
    args = parser.parse_args()
    return args


def import_ods_file(path):
    return pd.read_excel(path, engine="odf", index_col=0)


def load_config_yaml():
    dir_path = os.path.dirname(os.path.realpath(__file__))
    with open(dir_path + "/eval_config.yaml", "r") as stream:
        config = yaml.safe_load(stream)
        return config["color_mapping"], config["label_mapping"]


def get_data(path_to_parent_directory):
    file = os.path.join(path_to_parent_directory, "evaluation_summary.ods")
    data = import_ods_file(file)

    return data


def plot_pies(data, label_mapping):
    nrow = 1
    ncol = 4

    planners = list(data[list(data.keys())[0]].index)

    bar_num = 2
    bar_width = 0.01
    x_ticks_base = np.repeat(np.ceil(bar_num * bar_width) + bar_width, ncol) * np.arange(
        ncol)  # base for x position of bars

    fig, axs = plt.subplots(nrow, ncol)
    keys = ['Mean policy switching prob close obstacle distance',
            'Mean policy switching prob medium obstacle distance', 'Mean policy switching prob large obstacle distance',
            'Mean policy switching prob']

    for i, key in enumerate(keys):
        for aio_index, planner in enumerate(planners):
            axs[i].bar(x_ticks_base[i] + aio_index * bar_width, data[key][aio_index] * 100,
                       width=bar_width, label=label_mapping[planner], color=color_mapping[planner])

    for ax in axs:
        ax.grid('on')
        ax.tick_params(
            axis='x',
            which='both',
            bottom=False,
            top=False,
            labelbottom=False)
        ax.set_ylim(0, 40)

    axs[0].set_ylabel("Switching Probability", fontsize="13")

    labels = ['[0.0, 1.2) [m]', '[1.2, 2.5) [m]', '[2.5, ∞) [m]', 'Average']
    for i, label in enumerate(labels):
        axs[i].set_xlabel(label, fontsize="10", labelpad=10)

    plt.legend(loc='upper left', bbox_to_anchor=(1.05, 0.55), fontsize="13")

    # fig.suptitle('Dist. to Clostest Dyn. Obst.', fontsize=13)

    plt.tight_layout()
    plt.savefig("aio_switching_prob.png")
    plt.show()


if __name__ == "__main__":  # execute code
    args = parsing()
    path_to_parent_directory = args.path_to_parent_directory
    color_mapping, label_mapping = load_config_yaml()
    data = get_data(path_to_parent_directory)

    plot_pies(data, label_mapping)
