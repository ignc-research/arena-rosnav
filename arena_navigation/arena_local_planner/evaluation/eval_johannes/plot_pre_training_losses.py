import argparse

import numpy as np
from matplotlib import pyplot as plt


def plot_pretrain_losses(path: str):
    with np.load(path) as losses:
        train_loss = losses['train_loss']
        test_loss = losses['test_loss']

    test_range = list(range(len(test_loss)))
    train_range = list(range(1, len(train_loss) + 1))
    plt.plot(test_range, test_loss, label="test loss")
    plt.plot(train_range, train_loss, label="train loss")
    plt.legend(fontsize="15")
    plt.xlabel("Epochs", fontsize="17")
    plt.yticks(fontsize="15")
    plt.xticks(fontsize="15")
    plt.ylabel("Loss", fontsize="17")
    plt.grid()
    plt.tight_layout()
    plt.savefig("pretrain_losses.png")
    plt.show()


def parsing():
    parser = argparse.ArgumentParser(description='Plot pretrain loss')  # create parser object
    parser.add_argument('path_to_pretrain_npz_file', type=str, action='store',
                        help='path to pretrain npz file')
    args = parser.parse_args()
    return args


if __name__ == "__main__":  # execute code
    args = parsing()
    path_to_parent_directory = args.path_to_pretrain_npz_file
    plot_pretrain_losses(path_to_parent_directory)
