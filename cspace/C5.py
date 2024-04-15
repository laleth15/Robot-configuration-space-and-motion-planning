import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from helper_functions import *
import typing


def C5_func(q_grid: np.array, q_start: np.array, q_goal:np.array, c_path: typing.List[np.array]) -> typing.List[np.array]:
    """ Convert the path from indices of q_grid to actual robot configurations.

    Parameters
    ----------
    q_grid : np.array
        A R x 1 numpy array representing the grid over the angle-range. R is the resolution.
    q_start : np.array
        A 2 x 1 numpy array representing the start configuration of the robot in the format of [q1, q2].
    q_goal : np.array
        A 2 x 1 numpy array representing the goal configuration of the robot in the format of [q1, q2].
    c_path : typing.List[np.array]
        A list of 2 x 1 numpy array representing the path from the start configuration to the goal configuration using indices of q_grid.

    Returns
    -------
    typing.List[np.array]
        A list of 2 x 1 numpy array representing the path from the start configuration to the goal configuration using actual angle values.
        The first dimension is q1 and the second dimension is q2. Example: [ [q1_0 , q2_0], [q1_1, q2_1], .... ]
    """

    path = [q_start]
    for index in c_path:
        x, y = map(int, index)
        path.append(np.array([q_grid[x], q_grid[y]]))
    path.append(q_goal)
    return path