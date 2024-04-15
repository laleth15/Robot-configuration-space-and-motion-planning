import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from helper_functions import *

def C7_func(cspace: np.array) -> np.array:
    """Pad the configuration space by one grid cell.

    Parameters
    ----------
    cspace : np.array
        The origianl configuration space of the robot.

    Returns
    -------
    np.array
        The padded configuration space of the robot.
    """

    ### Insert your code below: ###
    space = cspace.copy()
    rows, cols = cspace.shape

    for row in range(rows):
        for col in range(cols):
            if cspace[row, col] == 1:
                for x in [-1, 0, 1]:
                    for y in [-1, 0, 1]:
                        if x == 0 and y == 0:
                            continue
                        newx, newy = row + x, col + y
                        if 0 <= newx < rows and 0 <= newy < cols:
                            space[newx, newy] = 1
    return space