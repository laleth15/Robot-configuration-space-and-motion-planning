import numpy as np
from shapely.geometry import Polygon as Polygon_shapely
import typing

def q2poly(robot: typing.Dict[str, typing.List[float]], q: typing.List[float]) -> typing.Tuple[np.array, np.array, np.array, np.array]:
    """ A function that takes in the robot's parameters and a configuration and 
    returns the vertices of the robot's links after transformation and the pivot points of the links after transformation

    Parameters
    ----------
    robot : typing.dict[str, typing.List[float]]
        A dictionary containing the robot's parameters
    q : typing.List[float]
        A 2-element list representing the configuration of the robot

    Returns
    -------
    typing.Tuple[np.array, np.array, np.array, np.array]
        np.array: 
            a numpy array representing the vertices of the first link of the robot after transformation
        np.array: 
            a numpy array representing the vertices of the second link of the robot after transformation
        np.array: 
            a numpy array representing the pivot point of the first link of the robot after transformation
        np.array: 
            a numpy array representing the pivot point of the second link of the robot after transformation
    """


    ### Insert your code below: ###


    c1, s1 = np.cos(q[0]), np.sin(q[0])
    c2, s2 = np.cos(q[0] + q[1]), np.sin(q[0] + q[1])

    pivot1 = np.array(robot["pivot1"])
    pivot2_relative = np.array(robot["pivot2"])

    pivot2 = pivot1 + np.array([c1 * pivot2_relative[0] - s1 * pivot2_relative[1], s1 * pivot2_relative[0] + c1 * pivot2_relative[1]])

    link1 = np.array(robot["link1"])
    link1_rotated = np.dot(link1, np.array([[c1, s1], [-s1, c1]]))
    shape1 = link1_rotated + pivot1

    link2 = np.array(robot["link2"])
    link2_rotated = np.dot(link2, np.array([[c2, s2], [-s2, c2]]))
    shape2 = link2_rotated + pivot2

    return shape1, shape2, pivot1, pivot2