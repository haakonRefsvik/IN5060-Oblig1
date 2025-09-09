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
        JPS motion plan function.

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

            jp_list = []
            for motion in self.motions:
                jp = self.jump(node, motion)
                # exists and not in CLOSED list
                if jp and jp.current not in CLOSED:
                    jp.parent = node.current
                    jp.h = self.h(jp, self.goal)
                    jp_list.append(jp)

            for jp in jp_list:
                # update OPEN list
                heapq.heappush(OPEN, jp)

                # goal found
                if jp == self.goal:
                    break

            CLOSED[node.current] = node
        return [], [], []

    def jump(self, node: Node, motion: Node):
        """
        Jumping search recursively.

        Parameters:
            node (Node): current node
            motion (Node): the motion that current node executes

        Returns:
            jump_point (Node): jump point or None if searching fails
        """
        # explore a new node
        new_node = node + motion
        new_node.parent = node.current
        new_node.h = self.h(new_node, self.goal)

        # hit the obstacle
        if new_node.current in self.obstacles:
            return None

        # goal found
        if new_node == self.goal:
            return new_node

        # 3D diagonal and oblique movements
        if motion.x and motion.y and motion.z:
            # All three dimensions - check all three orthogonal directions
            x_dir = Node((motion.x, 0, 0), None, 1, None)
            y_dir = Node((0, motion.y, 0), None, 1, None)
            z_dir = Node((0, 0, motion.z), None, 1, None)
            if self.jump(new_node, x_dir) or self.jump(new_node, y_dir) or self.jump(new_node, z_dir):
                return new_node
        elif motion.x and motion.y:
            # XY plane diagonal
            x_dir = Node((motion.x, 0, 0), None, 1, None)
            y_dir = Node((0, motion.y, 0), None, 1, None)
            if self.jump(new_node, x_dir) or self.jump(new_node, y_dir):
                return new_node
        elif motion.x and motion.z:
            # XZ plane diagonal
            x_dir = Node((motion.x, 0, 0), None, 1, None)
            z_dir = Node((0, 0, motion.z), None, 1, None)
            if self.jump(new_node, x_dir) or self.jump(new_node, z_dir):
                return new_node
        elif motion.y and motion.z:
            # YZ plane diagonal
            y_dir = Node((0, motion.y, 0), None, 1, None)
            z_dir = Node((0, 0, motion.z), None, 1, None)
            if self.jump(new_node, y_dir) or self.jump(new_node, z_dir):
                return new_node
            
        # if exists forced neighbor
        if self.detectForceNeighbor(new_node, motion):
            return new_node
        else:
            return self.jump(new_node, motion)

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

