import argparse  # get user input from terminal as arguments
import os

import matplotlib.pyplot as plt
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


def extract_plot_data(data_str: str):
    data_str = data_str.replace("[", "")
    data_str = data_str.replace("]", "")
    data_str = data_str.split(" ")
    return [float(i) * 100 for i in data_str]


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
    nrow = data.axes[0].size
    ncol = 4
    fig, axs = plt.subplots(nrow, ncol)
    keys = ['Mean model distribution close obstacle distance',
            'Mean model distribution medium obstacle distance', 'Mean model distribution large obstacle distance',
            'Mean model distribution']
    for aio_index in range(nrow):
        for i, key in enumerate(keys):
            # axs[i, aio_index].pie(extract_plot_data(data[key][aio_index]), labels=["Rosnav-DRL", "TEB"], startangle=90,
            #                       colors=["tab:red", "tab:green"], autopct='%1.0f%%', pctdistance=0.5,
            #                       labeldistance=1.1, textprops={'fontsize': 8}, radius=1.3)
            axs[aio_index, i].pie(extract_plot_data(data[key][aio_index]), startangle=90,
                                  colors=["tab:red", "tab:green"], autopct='%1.0f%%', pctdistance=0.5,
                                  labeldistance=1.1, textprops={'fontsize': 8}, radius=1.3)
        # axs[i//ncol,i%ncol].title.set_text(pie_labels[key])
    # plt.legend(loc='upper left', bbox_to_anchor=(1.1, 1.3), fontsize=14)
    # cols = ['[0.0,1.2][m]', '[1.2,2.5][m]', '[2.5,∞][m]', 'average']
    # rows = [label_mapping[planner] for planner in list(data[list(data.keys())[0]].index)]
    # for ax, col in zip(axs[0], cols):
    #     ax.set_title(col)
    # for ax, row in zip(axs[:, 0], rows):
    #     ax.yaxis.set_label_coords(-1.2, 0.4)
    #     ax.set_ylabel(row, rotation=0, size='large')
    plt.tight_layout()
    plt.savefig("model_distriubtion_pie.png")
    plt.show()


if __name__ == "__main__":  # execute code
    args = parsing()
    path_to_parent_directory = args.path_to_parent_directory
    color_mapping, label_mapping = load_config_yaml()
    data = get_data(path_to_parent_directory)

    plot_pies(data, label_mapping)
