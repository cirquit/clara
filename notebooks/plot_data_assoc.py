import matplotlib.pyplot as plt
import numpy             as np
import csv
from matplotlib.patches import Ellipse

from cppdebuginfo import yellow_cone_data, blue_cone_data, red_cone_data 

def draw_ellipse(position, covariance, ax=None, **kwargs):
    """Draw an ellipse with a given position and covariance"""
    ax = ax or plt.gca()

    # Convert covariance to principal axes
    if covariance.shape == (2, 2):
        U, s, Vt = np.linalg.svd(covariance)
        angle = np.degrees(np.arctan2(U[1, 0], U[0, 0]))
        width, height = 2 * np.sqrt(s)
    else:
        angle = 0
        width, height = 2 * np.sqrt(covariance)

    # Draw the Ellipse
    for nsig in range(1, 4):
        ax.add_patch(Ellipse(position, nsig * width, nsig * height,
                             angle, facecolor='black', **kwargs))

def plot_cluster_intrinsics(data, ax=None, color="black"):
    """Use the data with the following format to plot the map
    
    yellow_cone_data = np.array([
        np.array([[3.76783, -1.95591], [[3.15856e-07, 1.10833e-05], [1.10833e-05, 0.000417146]], [0.0372671]]),
        ...        mean_x     mean_y      cov_xx        cov_xy        cov_xy        cov_yy          weight (of cluster)
        ...
        ])

    """
    positions = data[:, 0]
    covariances = data[:, 1]
    weights = data[:, 2]
    pos_x = [e[0] for e in positions]
    pos_y = [e[1] for e in positions]

    ax.scatter(pos_x, pos_y, color=color)
    for pos, covar, w in zip(positions, covariances, weights):
        draw_ellipse(np.array(pos), np.array(covar), ax=ax, alpha=w[0])


def read_ground_truth(path):
    """Reads data from the csv file without a header, formatted like this:

        --, --, x, y, color

    """
    yellow_cone_ground_truth_x = np.array([])
    yellow_cone_ground_truth_y = np.array([])
    blue_cone_ground_truth_x   = np.array([])
    blue_cone_ground_truth_y   = np.array([])
    red_cone_ground_truth_x    = np.array([])
    red_cone_ground_truth_y    = np.array([])

    with open(path, 'rb') as csvfile:
        reader = csv.reader(csvfile, skipinitialspace=False, delimiter=',')
        for row in reader:
            x = row[2]
            y = row[3]
            color = row[4]
            if color == ' 0':
                yellow_cone_ground_truth_x = np.append(yellow_cone_ground_truth_x, x);
                yellow_cone_ground_truth_y = np.append(yellow_cone_ground_truth_y, y); 
            elif color == ' 1':
                blue_cone_ground_truth_x = np.append(blue_cone_ground_truth_x, x); 
                blue_cone_ground_truth_y = np.append(blue_cone_ground_truth_y, y);
            else:
                red_cone_ground_truth_x = np.append(red_cone_ground_truth_x, x); 
                red_cone_ground_truth_y = np.append(red_cone_ground_truth_y, y); 

    return yellow_cone_ground_truth_x, yellow_cone_ground_truth_y,
           blue_cone_ground_truth_x,   blue_cone_ground_truth_y,
           red_cone_ground_truth_x,    red_cone_ground_truth_y



path = 'example-data/wemding-map-ground-truth-cones-d-a-x-y-c-t.csv'
yellow_cone_ground_truth_x,   yellow_cone_ground_truth_y,
    blue_cone_ground_truth_x, blue_cone_ground_truth_y,
    red_cone_ground_truth_x,  red_cone_ground_truth_y = read_ground_truth(path)

# create a big figure
plt.figure(figsize=(12,12))
ax = plt.gca()

# plot the ground truth data
ax.scatter(yellow_cone_ground_truth_x, yellow_cone_ground_truth_y, color='black')
ax.scatter(blue_cone_ground_truth_x, blue_cone_ground_truth_y, color='black')
ax.scatter(red_cone_ground_truth_x, red_cone_ground_truth_y, color='black')

# plot the calculated data
plot_cluster_intrinsics(yellow_cone_data, ax=ax, color='#D5D106')
# plot_cluster_intrinsics(blue_cone_data, ax=ax, color='#898AEF')
# plot_cluster_intrinsics(red_cone_data, ax=ax)