"""
@file: theta_star.py
@breif: Theta* motion planning
@author: Yang Haodong, Wu Maojia
@update: 2024.6.23
"""
import heapq

from .a_star import AStar
from python_motion_planning.utils import Env, Node, Grid


class ThetaStar(AStar):
    """
    Class for Theta* motion planning.

    Parameters:
        start (tuple):
            start point coordinate
        goal (tuple):
            goal point coordinate
        env (Grid):
            environment
        heuristic_type (str):
            heuristic function type

    Examples:
        >>> import python_motion_planning as pmp
        >>> planner = pmp.ThetaStar((5, 5), (45, 25), pmp.Grid(51, 31))
        >>> cost, path, expand = planner.plan()
        >>> planner.plot.animation(path, str(planner), cost, expand)  # animation
        >>> planner.run()       # run both planning and animation

    References:
        [1] Theta*: Any-Angle Path Planning on Grids
        [2] Any-angle path planning on non-uniform costmaps
    """
    def __init__(self, start: tuple, goal: tuple, env: Grid, heuristic_type: str = "euclidean") -> None:
        super().__init__(start, goal, env, heuristic_type)

    def __str__(self) -> str:
        return "Theta*"

    def plan(self) -> tuple:
        """
        Theta* motion plan function.

        Returns:
            cost (float): path cost
            path (list): planning path
            expand (list): all nodes that planner has searched
        """
        # OPEN list (priority queue) and CLOSED list (hash table)
        OPEN = []
        heapq.heappush(OPEN, self.start)
        CLOSED = dict()

        while OPEN:
            node = heapq.heappop(OPEN)

            # exists in CLOSED list
            if node.current in CLOSED:
                continue

            # goal found
            if node == self.goal:
                CLOSED[node.current] = node
                cost, path = self.extractPath(CLOSED)
                return cost, path, list(CLOSED.values())

            for node_n in self.getNeighbor(node):                
                # exists in CLOSED list
                if node_n.current in CLOSED:
                    continue
                
                # path1
                node_n.parent = node.current
                node_n.h = self.h(node_n, self.goal)

                node_p = CLOSED.get(node.parent)

                if node_p:
                    self.updateVertex(node_p, node_n)

                # goal found
                if node_n == self.goal:
                    heapq.heappush(OPEN, node_n)
                    break
                
                # update OPEN list
                heapq.heappush(OPEN, node_n)

            CLOSED[node.current] = node
        return [], [], []

    def updateVertex(self, node_p: Node, node_c: Node) -> None:
        """
        Update extend node information with current node's parent node.

        Parameters:
            node_p (Node): parent node
            node_c (Node): current node
        """
        if self.lineOfSight(node_c, node_p):
            # path 2
            if node_p.g + self.dist(node_c, node_p) <= node_c.g:
                node_c.g = node_p.g + self.dist(node_c, node_p)
                node_c.parent = node_p.current

    def lineOfSight(self, node1: Node, node2: Node) -> bool:
        """
        Judge collision when moving from node1 to node2 using 3D Bresenham.

        Parameters:
            node1 (Node): start node
            node2 (Node): end node

        Returns:
            line_of_sight (bool): True if line of sight exists ( no collision ) else False
        """
        if node1.current in self.obstacles or node2.current in self.obstacles:
            return False

        x1, y1, z1 = node1.current
        x2, y2, z2 = node2.current

        # Boundary checks
        if x1 < 0 or x1 >= self.env.x_range or y1 < 0 or y1 >= self.env.y_range or z1 < 0 or z1 >= self.env.z_range:
            return False
        if x2 < 0 or x2 >= self.env.x_range or y2 < 0 or y2 >= self.env.y_range or z2 < 0 or z2 >= self.env.z_range:
            return False

        # 3D Bresenham line algorithm - improved version
        dx = abs(x2 - x1)
        dy = abs(y2 - y1) 
        dz = abs(z2 - z1)
        
        x_inc = 1 if x2 > x1 else -1
        y_inc = 1 if y2 > y1 else -1
        z_inc = 1 if z2 > z1 else -1
        
        # Find the driving axis (the one with maximum change)
        if dx >= dy and dx >= dz:
            # X is driving axis
            err_1 = 2 * dy - dx
            err_2 = 2 * dz - dx
            x, y, z = x1, y1, z1
            
            while x != x2:
                if (int(x), int(y), int(z)) in self.obstacles:
                    return False
                    
                if err_1 > 0:
                    y += y_inc
                    err_1 -= 2 * dx
                if err_2 > 0:
                    z += z_inc
                    err_2 -= 2 * dx
                    
                err_1 += 2 * dy
                err_2 += 2 * dz
                x += x_inc
                
        elif dy >= dx and dy >= dz:
            # Y is driving axis
            err_1 = 2 * dx - dy
            err_2 = 2 * dz - dy
            x, y, z = x1, y1, z1
            
            while y != y2:
                if (int(x), int(y), int(z)) in self.obstacles:
                    return False
                    
                if err_1 > 0:
                    x += x_inc
                    err_1 -= 2 * dy
                if err_2 > 0:
                    z += z_inc
                    err_2 -= 2 * dy
                    
                err_1 += 2 * dx
                err_2 += 2 * dz
                y += y_inc
                
        else:
            # Z is driving axis
            err_1 = 2 * dx - dz
            err_2 = 2 * dy - dz
            x, y, z = x1, y1, z1
            
            while z != z2:
                if (int(x), int(y), int(z)) in self.obstacles:
                    return False
                    
                if err_1 > 0:
                    x += x_inc
                    err_1 -= 2 * dz
                if err_2 > 0:
                    y += y_inc
                    err_2 -= 2 * dz
                    
                err_1 += 2 * dx
                err_2 += 2 * dy
                z += z_inc
        
        # Check the final point
        if (int(x2), int(y2), int(z2)) in self.obstacles:
            return False
            
        return True
