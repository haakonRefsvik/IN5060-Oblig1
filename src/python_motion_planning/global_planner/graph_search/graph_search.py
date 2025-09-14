"""
@file: graph_search.py
@breif: Base class for planner based on graph searching
@author: Winter
@update: 2023.1.13
"""
import math
from python_motion_planning.utils import Env, Node, Planner, Grid


class GraphSearcher(Planner):
    """
    Base class for planner based on graph searching.

    Parameters:
        start (tuple): start point coordinate
        goal (tuple): goal point coordinate
        env (Grid): environment
        heuristic_type (str): heuristic function type
    """
    def __init__(self, start: tuple, goal: tuple, env: Grid, heuristic_type: str="euclidean") -> None:
        super().__init__(start, goal, env)
        # heuristic type
        self.heuristic_type = heuristic_type
        # allowed motions
        self.motions = self.env.motions
        # obstacles
        self.obstacles = self.env.obstacles

    def h(self, node: Node, goal: Node) -> float:
        """
        Calculate heuristic.

        Parameters:
            node (Node): current node
            goal (Node): goal node

        Returns:
            h (float): heuristic function value of node
        """

        if self.heuristic_type == "manhattan":
            return abs(goal.x - node.x) + abs(goal.y - node.y) + abs(goal.z - node.z)

        elif self.heuristic_type == "euclidean":
            return math.sqrt((goal.x - node.x)**2 + (goal.y - node.y)**2 + (goal.z - node.z)**2)

    def cost(self, node1: Node, node2: Node) -> float:
        """
        Calculate motion cost with altitude reward/penalty.
        Lower z → higher cost, higher z → lower cost.
        """
        if self.isCollision(node1, node2):
            return float("inf")

        base_cost = self.dist(node1, node2)

        ## If the altitude (z) is lower than 5, then double the motion cost
        if node2.z < 5:
            altitude_factor = 2.0
        else:
            altitude_factor = 1.0

        return base_cost * altitude_factor

    def isCollision(self, node1, node2):
        x1, y1, z1 = node1.current
        x2, y2, z2 = node2.current
        dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
        steps = max(abs(dx), abs(dy), abs(dz))
        for i in range(1, steps + 1):
            x = x1 + int(round(dx * i / steps))
            y = y1 + int(round(dy * i / steps))
            z = z1 + int(round(dz * i / steps))
            if (x, y, z) in self.obstacles:
                return True
        return False

