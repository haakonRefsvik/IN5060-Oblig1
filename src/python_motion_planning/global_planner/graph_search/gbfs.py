"""
@file: gbfs.py
@breif: Greedy Best First Search motion planning
@author: Yang Haodong, Wu Maojia
@update: 2024.6.23
"""
import heapq

from .a_star import AStar
from python_motion_planning.utils import Env, Grid


class GBFS(AStar):
    """
    Class for Greedy Best First Search.

    Parameters:
        start (tuple): start point coordinate
        goal (tuple): goal point coordinate
        env (Grid): environment
        heuristic_type (str): heuristic function type

    Examples:
        >>> import python_motion_planning as pmp
        >>> planner = pmp.GBFS((5, 5), (45, 25), pmp.Grid(51, 31))
        >>> cost, path, expand = planner.plan()     # planning results only
        >>> planner.plot.animation(path, str(planner), cost, expand)  # animation
        >>> planner.run()       # run both planning and animation
    """
    def __init__(self, start: tuple, goal: tuple, env: Grid, heuristic_type: str = "euclidean") -> None:
        super().__init__(start, goal, env, heuristic_type)
    
    def __str__(self) -> str:
        return "Greedy Best First Search(GBFS)"

    def plan(self) -> tuple:
        OPEN = []
        heapq.heappush(OPEN, self.start)
        CLOSED = dict()

        while OPEN:
            node = heapq.heappop(OPEN)

            if node.current in CLOSED:
                continue

            if node == self.goal:
                CLOSED[node.current] = node
                cost, path = self.extractPath(CLOSED)
                return cost, path, list(CLOSED.values())

            for node_n in self.getNeighbor(node):
                if node_n.current in self.obstacles:
                    continue
                if node_n.current in CLOSED:
                    continue

                node_n.parent = node.current

                # normal heuristic
                h_val = self.h(node_n, self.goal)

                # ✅ altitude penalty for z < 5
                if node_n.z is not None and node_n.z < 5:
                    h_val *= 2.0   # increase "costliness" in priority

                node_n.h = h_val
                node_n.g = 0  # still greedy best-first (ignores true g)

                if node_n == self.goal:
                    heapq.heappush(OPEN, node_n)
                    break

                heapq.heappush(OPEN, node_n)

            CLOSED[node.current] = node
        return [], [], []
