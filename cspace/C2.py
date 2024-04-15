import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from shapely.geometry import Polygon as Polygon_shapely
from helper_functions import *
from q2poly import q2poly
import typing

def C2_func(robot: typing.Dict[str, typing.List[float]], cspace: np.array, obstacles: typing.List[Polygon],q_grid: np.array) -> np.array:
    """Create the configuration space for the robot with the given obstacles in the given empty cspace array.

    Parameters
    ----------
    robot : typing.Dict[str, typing.List[float]]
        A dictionary containing the robot's parameters
    cspace : np.array
        An empty 2D numpy array
    obstacles : typing.List[Polygon]
        A list of polygons representing the obstacles
    q_grid : np.array
        A R x 1 numpy array representing the grid over the angle-range. R is the resolution.

    Returns
    -------
    np.array
        A 2D numpy array representing the updated configuration space. The first dimension is q1 and the second dimension is q2. Example: [q1, q2]
    """

    cspace = np.zeros((len(q_grid), len(q_grid)))

    for i in range(len(q_grid)):
        for j in range(len(q_grid)):
            shape1, shape2, _, _ = q2poly(robot, [q_grid[i], q_grid[j]])
            poly1 = Polygon_shapely(shape1)
            poly2 = Polygon_shapely(shape2)
            for obstacle in obstacles:
                obs = Polygon_shapely(obstacle)
                if poly1.intersects(obs) or poly2.intersects(obs):
                    cspace[i, j] = 1
                    break
    return cspace
