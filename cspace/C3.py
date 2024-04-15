import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from helper_functions import *
import typing


def C3_func(robot: typing.Dict[str, typing.List[float]], cspace: np.array,q_grid: np.array, q_goal: np.array) -> np.array:
    """Create a new 2D array that shows the distance from each point in the configuration space to the goal configuration.

    Parameters
    ----------
    robot : typing.Dict[str, typing.List[float]]
        A dictionary containing the robot's parameters
    cspace : np.array
        The configuration space of the robot given by C2. The first dimension is q1 and the second dimension is q2. Example: [q1, q2]
    q_grid : np.array
        A R x 1 numpy array representing the grid over the angle-range. R is the resolution.
    q_goal : np.array
        A 2 x 1 numpy array representing the goal configuration of the robot in the format of [q1, q2].

    Returns
    -------
    np.array
       A 2D numpy array representing the distance from each cell in the configuration space to the goal configuration. 
       The first dimension is q1 and the second dimension is q2. Example: [q1, q2]
    """

    x = np.argmin(np.abs(q_grid - q_goal[0]))
    y = np.argmin(np.abs(q_grid - q_goal[1]))

    distances = np.full(cspace.shape, np.inf)
    distances[cspace == 1] = 1  
    distances[x, y] = 2  
    queue = [(x, y)]
    movements = [(a, b) for a in [-1, 0, 1] for b in [-1, 0, 1] if a != 0 or b != 0]

    queue = [(x, y)]
    while queue:
        w, z = queue.pop(0)
        for a, b in movements:
            newx, newy = w + a, z + b
            if 0 <= newx < cspace.shape[0] and 0 <= newy < cspace.shape[1] and cspace[newx, newy] == 0:
                if distances[newx, newy] > distances[w, z] + 1:
                    distances[newx, newy] = distances[w, z] + 1
                    queue.append((newx, newy))
    distances[distances == np.inf] = 0
    return distances