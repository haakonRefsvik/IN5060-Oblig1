"""
@file: jps.py
@breif: Jump Point Search motion planning
@author: Yang Haodong, Wu Maojia
@update: 2024.6.23
"""
import heapq

from .a_star import AStar
from python_motion_planning.utils import Env, Node, Grid

class JPS(AStar):
    """
    Class for JPS motion planning.

    Parameters:
        start (tuple): start point coordinate
        goal (tuple): goal point coordinate
        env (Grid): environment
        heuristic_type (str): heuristic function type

    Examples:
        >>> import python_motion_planning as pmp
        >>> planner = pmp.JPS((5, 5), (45, 25), pmp.Grid(51, 31))
        >>> cost, path, expand = planner.plan()     # planning results only
        >>> planner.plot.animation(path, str(planner), cost, expand)  # animation
        >>> planner.run()       # run both planning and animation

    References:
        [1] Online Graph Pruning for Pathfinding On Grid Maps
    """
    def __init__(self, start: tuple, goal: tuple, env: Grid, heuristic_type: str = "euclidean") -> None:
        super().__init__(start, goal, env, heuristic_type)
    
    def __str__(self) -> str:
        return "Jump Point Search(JPS)"

    def plan(self) -> tuple:
        """
        Optimized JPS plan function.
        Returns:
            cost (float), path (list), expand (list)
        """
        OPEN = []
        self.start.g = 0
        self.start.h = self.h(self.start, self.goal)
        self.start.f = self.start.g + self.start.h
        heapq.heappush(OPEN, (self.start.f, self.start))
        CLOSED = dict()

        while OPEN:
            _, node = heapq.heappop(OPEN)

            if node.current in CLOSED:
                continue

            CLOSED[node.current] = node

            if node == self.goal:
                cost, path = self.extractPath(CLOSED)
                return cost, path, list(CLOSED.values())

            for motion in self.motions:
                jp = self.jump(node, motion)
                if jp and jp.current not in CLOSED:
                    step_cost = self.cost(node, jp)
                    jp.g = node.g + step_cost
                    jp.h = self.h(jp, self.goal)
                    jp.f = jp.g + jp.h
                    jp.parent = node.current
                    heapq.heappush(OPEN, (jp.f, jp))

        return [], [], []

    def jump(self, node: Node, motion: Node):
        """
        Iterative jump in 3D. Stops at forced neighbor, obstacle, or goal.
        """
        current = Node(node.current, node.parent, node.g, node.h)

        while True:
            # Step in motion direction
            current = current + motion

            # Stop if outside grid or obstacle
            x, y, z = current.current
            if not (0 <= x < self.env.x_range and 0 <= y < self.env.y_range and 0 <= z < self.env.z_range):
                return None
            if current.current in self.obstacles:
                return None

            # Goal found
            if current == self.goal:
                return current

            # Forced neighbor detection
            if self.detectForceNeighbor(current, motion):
                return current

            # Stop stepping if motion is 1D along any axis
            if abs(motion.x) + abs(motion.y) + abs(motion.z) == 1:
                return current


    def detectForceNeighbor(self, node, motion):
        """
        Detect forced neighbor of node in 3D space.

        Parameters:
            node (Node): current node
            motion (Node): the motion that current node executes

        Returns:
            flag (bool): True if current node has forced neighbor else False
        """
        x, y, z = node.current
        x_dir, y_dir, z_dir = motion.current
        
        # Horizontal movement (x-direction)
        if x_dir and not y_dir and not z_dir:
            for dy in [-1, 1]:
                for dz in [-1, 1]:
                    if (x, y + dy, z + dz) in self.obstacles and \
                       (x + x_dir, y + dy, z + dz) not in self.obstacles:
                        return True
        
        # Vertical movement (y-direction)
        if not x_dir and y_dir and not z_dir:
            for dx in [-1, 1]:
                for dz in [-1, 1]:
                    if (x + dx, y, z + dz) in self.obstacles and \
                       (x + dx, y + y_dir, z + dz) not in self.obstacles:
                        return True
        
        # Depth movement (z-direction)
        if not x_dir and not y_dir and z_dir:
            for dx in [-1, 1]:
                for dy in [-1, 1]:
                    if (x + dx, y + dy, z) in self.obstacles and \
                       (x + dx, y + dy, z + z_dir) not in self.obstacles:
                        return True
        
        # Diagonal movements in XY plane
        if x_dir and y_dir and not z_dir:
            if (x - x_dir, y, z) in self.obstacles and \
               (x - x_dir, y + y_dir, z) not in self.obstacles:
                return True
            if (x, y - y_dir, z) in self.obstacles and \
               (x + x_dir, y - y_dir, z) not in self.obstacles:
                return True
        
        # Diagonal movements in XZ plane
        if x_dir and not y_dir and z_dir:
            if (x - x_dir, y, z) in self.obstacles and \
               (x - x_dir, y, z + z_dir) not in self.obstacles:
                return True
            if (x, y, z - z_dir) in self.obstacles and \
               (x + x_dir, y, z - z_dir) not in self.obstacles:
                return True
        
        # Diagonal movements in YZ plane
        if not x_dir and y_dir and z_dir:
            if (x, y - y_dir, z) in self.obstacles and \
               (x, y - y_dir, z + z_dir) not in self.obstacles:
                return True
            if (x, y, z - z_dir) in self.obstacles and \
               (x, y + y_dir, z - z_dir) not in self.obstacles:
                return True
        
        # 3D diagonal movements (all three directions)
        if x_dir and y_dir and z_dir:
            # Check several key forced neighbor patterns for 3D diagonal movement
            if (x - x_dir, y, z) in self.obstacles and \
               (x - x_dir, y + y_dir, z + z_dir) not in self.obstacles:
                return True
            if (x, y - y_dir, z) in self.obstacles and \
               (x + x_dir, y - y_dir, z + z_dir) not in self.obstacles:
                return True
            if (x, y, z - z_dir) in self.obstacles and \
               (x + x_dir, y + y_dir, z - z_dir) not in self.obstacles:
                return True
        
        return False

