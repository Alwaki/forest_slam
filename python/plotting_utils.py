import numpy as np
from matplotlib import pyplot as plt

def plot_gt_trees(filename, show_now = 0):
    """ Plots all ground truth tree positions

    Args:
        filename (string): file system path
        main_tree (int, optional): The tree that should be considered the center point of the data. Defaults to 1.
        show_now (bool, optional): indicates if plot should be drawn and shown immediately. Defaults to 0.
    """
    data = np.loadtxt(filename, delimiter='\t', dtype=float)
    plt.plot(
                data[:,0],
                data[:,1],
                "*",
                markerfacecolor="k",
                markeredgecolor="k",
                markersize=12,
            )
    for i, _ in enumerate(data):
        plt.annotate(str(i+1), (data[i, 0], data[i, 1]), fontsize=12)
    if show_now:
        plt.show()

plot_gt_trees("data.txt", 1)