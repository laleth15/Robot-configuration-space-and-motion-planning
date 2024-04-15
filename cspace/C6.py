import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from helper_functions import *
from q2poly import q2poly
import shapely
from shapely.geometry import Polygon as Polygon_shapely
from shapely import MultiPoint
import typing

from C1 import C1_func




def C6_func(robot: typing.Dict[str, typing.List[float]], q_path: typing.List[np.array], obstacles: typing.List[Polygon]) -> int:
    """Calculate the number of collisions that occur along the path.
    
    Parameters
    ----------
    robot : typing.Dict[str, typing.List[float]]
        A dictionary containing the robot's parameters.
    q_path : typing.List[np.array]
       A list of 2 x 1 numpy array representing the path from the start configuration to the goal configuration using actual angle values.
    obstacles : typing.List[Polygon]
        A list of polygons representing the obstacles.

    Returns
    -------
    int
        The number of collisions that occur along the path.
    """

    num_collisions = 0

    for i in range(len(q_path) - 1):
        x = q_path[i]
        y = q_path[i + 1]
        
        a, b, _, _ = q2poly(robot, x)
        d, c, _, _ = q2poly(robot, y)
        
        link1_volume = Polygon_shapely(np.concatenate([a, d])).convex_hull
        link2_volume = Polygon_shapely(np.concatenate([b, c])).convex_hull
        
        for obs in obstacles:
            object = Polygon_shapely(obs)
            if link1_volume.intersects(object) or link2_volume.intersects(object):
                num_collisions += 1
                break

    return num_collisions