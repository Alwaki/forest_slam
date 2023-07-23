from matplotlib import pyplot as plt, colors
import numpy as np

def plot(X, labels, parameters=None, ground_truth=False, ax=None):
    _, ax = plt.subplots()
    # Black removed and is used for noise instead.
    unique_labels = set(labels)
    unique_labels.discard(-1)
    colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
    for k, col in zip(unique_labels, colors):
        print("new value")
        class_index = np.where(labels == k)[0]
        print(len(class_index))
        for ci in class_index:
            ax.plot(
                X[ci, 0],
                X[ci, 1],
                "o",
                markerfacecolor=tuple(col),
                markeredgecolor="k",
                markersize=7
            )
    plt.show()

def plot_landmarks(landmarks, slice_count, show_now = 0):
    """ Plots all landmarks as circles representing covariance (uncertainty). 
        Estimated position is center of circle.

    Args:
        landmarks (list(LandmarkObject)): List containing found landmarks (represented as multivariate Gaussians)
        slice_count (int): Number of slices used in associating landmarks
        show_now (bool, optional): indicates if plot should be drawn and shown immediately. Defaults to 0.
    """
    t = np.linspace(0, 2*np.pi, 30)
    cmap = plt.cm.Reds
    norm = colors.Normalize(vmin=0.0, vmax=1)
    for landmark in landmarks:
        mean = landmark.getPos()
        cov = landmark.getCov()
        plt.plot(mean[0]+cov[0]*np.cos(t), mean[1]+cov[1]*np.sin(t), c=cmap(norm(landmark.getObsCount()/slice_count)), linewidth=2.5)
    if show_now:
        plt.grid()
        plt.show()