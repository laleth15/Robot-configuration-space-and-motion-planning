import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from helper_functions import *
import typing

def C4_func(distances: np.array,q_grid: np.array, q_start: np.array) -> typing.List[np.array]:
    """Using the distance array from C3, find the optimal path from the start configuration to the goal configuration (zero value).

    Parameters
    ----------
    distances : np.array
        A 2D numpy array representing the distance from each cell in the configuration space to the goal configuration.
        This is given by C3 
    q_grid : np.array
        A R x 1 numpy array representing the grid over the angle-range. R is the resolution.
    q_start : np.array
        A 2 x 1 numpy array representing the start configuration of the robot in the format of [q1, q2].

    Returns
    -------
    typing.List[np.array]
        A list of 2 x 1 numpy array representing the path from the start configuration to the goal configuration using indices of q_grid.
        Example: [ [q1_0 , q2_0], [q1_1, q2_1], .... ]
    """
    
    x = np.argmin(np.abs(q_grid - q_start[0]))
    y = np.argmin(np.abs(q_grid - q_start[1]))

    path = [(x, y)]

    neighbours = [(a, b) for a in [-1, 0, 1] for b in [-1, 0, 1] if a != 0 or b != 0]
    price = distances[x, y]

    while price != 2 and price != 0:
        a, b = path[-1]
        step = False
        for u, v in neighbours:
            newx, newy = a + u, b + v
            if 0 <= newx < distances.shape[0] and 0 <= newy < distances.shape[1]:
                if distances[newx, newy] != 1:
                    if distances[newx, newy] == 2 or distances[newx, newy] < price:
                        path.append((newx, newy))
                        price = distances[newx, newy]
                        step = True
                        break
        if not step:
            break
    return path