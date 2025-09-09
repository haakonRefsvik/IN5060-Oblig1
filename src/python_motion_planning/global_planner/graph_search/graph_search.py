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
        dx, dy, dz = goal.x - node.x, goal.y - node.y, goal.z - node.z
        if self.heuristic_type == "manhattan":
            return abs(dx) + abs(dy) + abs(dz)
        elif self.heuristic_type == "euclidean":
            return math.sqrt(dx**2 + dy**2 + dz**2)

    def cost(self, node1: Node, node2: Node) -> float:
        """
        Calculate cost for this motion.

        Parameters:
            node1 (Node): node 1
            node2 (Node): node 2

        Returns:
            cost (float): cost of this motion
        """
        if self.isCollision(node1, node2):
            return float("inf")
        return self.dist(node1, node2)

    def isCollision(self, node1: Node, node2: Node) -> bool:
        """
        Judge collision when moving from node1 to node2 in 3D.

        Parameters:
            node1 (Node): node 1
            node2 (Node): node 2

        Returns:
            collision (bool): True if collision exists else False
        """
        # Direct collisions
        if node1.current in self.obstacles or node2.current in self.obstacles:
            return True

        x1, y1, z1 = node1.x, node1.y, node1.z
        x2, y2, z2 = node2.x, node2.y, node2.z

        dx, dy, dz = x2 - x1, y2 - y1, z2 - z1

        # If moving diagonally (any two or three axes at once), check all "shared faces/edges"
        intermediate_points = []

        # Check x-y plane between nodes
        if dx != 0 and dy != 0:
            intermediate_points.append((min(x1, x2), min(y1, y2), z1))
            intermediate_points.append((max(x1, x2), max(y1, y2), z1))

        # Check x-z plane between nodes
        if dx != 0 and dz != 0:
            intermediate_points.append((min(x1, x2), y1, min(z1, z2)))
            intermediate_points.append((max(x1, x2), y1, max(z1, z2)))

        # Check y-z plane between nodes
        if dy != 0 and dz != 0:
            intermediate_points.append((x1, min(y1, y2), min(z1, z2)))
            intermediate_points.append((x1, max(y1, y2), max(z1, z2)))

        # If moving along all three axes (true 3D diagonal), add the central cube corners
        if dx != 0 and dy != 0 and dz != 0:
            intermediate_points.append((min(x1, x2), min(y1, y2), min(z1, z2)))
            intermediate_points.append((max(x1, x2), max(y1, y2), max(z1, z2)))

        # Check for collisions on these intermediate points
        for pt in intermediate_points:
            if pt in self.obstacles:
                return True

        return False
