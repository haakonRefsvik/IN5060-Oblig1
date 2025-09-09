"""
@file: env.py
@breif: 2-dimension environment
@author: Winter
@update: 2023.1.13
"""
from math import sqrt
from abc import ABC, abstractmethod
from scipy.spatial import cKDTree
import numpy as np

from .node import Node

class Env(ABC):
    """
    Class for building 2-d workspace of robots.

    Parameters:
        x_range (int): x-axis range of enviroment
        y_range (int): y-axis range of environmet
        eps (float): tolerance for float comparison

    Examples:
        >>> from python_motion_planning.utils import Env
        >>> env = Env(30, 40)
    """
    def __init__(self, x_range: int, y_range: int, z_range: int = None, eps: float = 1e-6) -> None:
        # size of environment
        self.x_range = x_range  
        self.y_range = y_range
        self.z_range = z_range
        self.eps = eps

    @property
    def grid_map(self) -> set:
        if self.z_range is not None:
            return {(i, j, k) for i in range(self.x_range) for j in range(self.y_range) for k in range(self.z_range)}
        else:
            return {(i, j) for i in range(self.x_range) for j in range(self.y_range)}

    @abstractmethod
    def init(self) -> None:
        pass

class Grid(Env):
    """
    Class for discrete 3-d grid map.

    Parameters:
        x_range (int): x-axis range of enviroment
        y_range (int): y-axis range of environmet
        z_range (int): z-axis range of environment
    """
    def __init__(self, x_range: int, y_range: int, z_range: int = None) -> None:
        super().__init__(x_range, y_range, z_range)
        # allowed motions (26 neighbors in 3D, 8 in 2D)
        if self.z_range is not None:
            self.motions = []
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    for dz in [-1, 0, 1]:
                        if dx == dy == dz == 0:
                            continue
                        cost = sqrt(dx**2 + dy**2 + dz**2)
                        self.motions.append(Node((dx, dy, dz), None, cost, None))
        else:
            self.motions = [Node((-1, 0), None, 1, None), Node((-1, 1),  None, sqrt(2), None),
                            Node((0, 1),  None, 1, None), Node((1, 1),   None, sqrt(2), None),
                            Node((1, 0),  None, 1, None), Node((1, -1),  None, sqrt(2), None),
                            Node((0, -1), None, 1, None), Node((-1, -1), None, sqrt(2), None)]
        # obstacles
        self.obstacles = None
        self.obstacles_tree = None
        self.init()

    def init(self) -> None:
        """
        Initialize grid map.
        """
        x, y = self.x_range, self.y_range
        z = self.z_range if self.z_range is not None else None
        obstacles = set()

        if z == None: 
            return

        for i in range(x):
            for k in range(z):
                obstacles.add((i, 0, k))
                obstacles.add((i, y - 1, k))
        for j in range(y):
            for k in range(z):
                obstacles.add((0, j, k))
                obstacles.add((x - 1, j, k))
        for i in range(x):
            for j in range(y):
                obstacles.add((i, j, 0))
                obstacles.add((i, j, z - 1))

        self.update(obstacles)

    def update(self, obstacles):
        self.obstacles = obstacles 
        self.obstacles_tree = cKDTree(np.array(list(obstacles)))


class Map(Env):
    """
    Class for continuous 2-d map.

    Parameters:
        x_range (int): x-axis range of enviroment
        y_range (int): y-axis range of environmet
    """
    def __init__(self, x_range: int, y_range: int, z_range: int = None) -> None:
        super().__init__(x_range, y_range, z_range)
        self.boundary = None
        self.obs_circ = None
        self.obs_rect = None
        self.init()

    def init(self):
        """
        Initialize map.
        """
        x, y = self.x_range, self.y_range
        z = self.z_range if self.z_range is not None else None

        # boundary of environment
        if z is not None:
            self.boundary = [
                [0, 0, 1, y, z],
                [0, y, x, 1, z],
                [1, 0, x, 1, z]
            ]
        else:
            self.boundary = [
                [0, 0, 1, y],
                [0, y, x, 1],
                [1, 0, x, 1]
            ]
        self.obs_rect = []
        self.obs_circ = []

    def update(self, boundary=None, obs_circ=None, obs_rect=None):
        self.boundary = boundary if boundary else self.boundary
        self.obs_circ = obs_circ if obs_circ else self.obs_circ
        self.obs_rect = obs_rect if obs_rect else self.obs_rect
