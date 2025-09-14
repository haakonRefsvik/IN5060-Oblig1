import heapq
from .a_star import AStar
from python_motion_planning.utils import Node, Grid

class JPS(AStar):
    """
    Iterative, efficient 3D Jump Point Search planner.
    """

    def __init__(self, start: tuple, goal: tuple, env: Grid, heuristic_type: str = "euclidean"):
        super().__init__(start, goal, env, heuristic_type)

    def __str__(self):
        return "Jump Point Search(JPS)"

    def plan(self):
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

            # Explore jump points from current node
            for motion in self.prune_motions(node):
                jp, g_inc = self.jump(node, motion)
                if jp and (jp.current not in CLOSED or jp.g > node.g + g_inc):
                    jp.g = node.g + g_inc
                    jp.h = self.h(jp, self.goal)
                    jp.f = jp.g + jp.h
                    jp.parent = node.current
                    heapq.heappush(OPEN, (jp.f, jp))

        return [], [], []

    def jump(self, node: Node, motion: Node):
        """
        Iterative jump along the motion direction.
        Returns a jump point node and the incremental cost from `node`.
        Properly accumulates altitude-adjusted cost.
        """
        dx, dy, dz = motion.current
        x, y, z = node.current
        g_inc = 0

        while True:
            nx, ny, nz = x + dx, y + dy, z + dz

            # Out of bounds
            if not (0 <= nx < self.env.x_range and 0 <= ny < self.env.y_range and 0 <= nz < self.env.z_range):
                return None, None
            # Obstacle collision
            if (nx, ny, nz) in self.obstacles:
                return None, None

            # Compute step cost from previous to new node
            prev_node = Node((x, y, z), None, 0, 0)
            next_node = Node((nx, ny, nz), None, 0, 0)
            step_cost = self.cost(prev_node, next_node)
            if step_cost == float("inf"):
                return None, None

            g_inc += step_cost

            # Move to next step
            x, y, z = nx, ny, nz
            current = Node((x, y, z), node.current, 0, 0)

            # Goal found
            if current == self.goal:
                return current, g_inc

            # Forced neighbor triggers jump point
            if self.detectForceNeighbor(current, motion):
                return current, g_inc

            # If moving along a single axis, stop here (1D motion)
            if abs(dx) + abs(dy) + abs(dz) == 1:
                return current, g_inc

            # For diagonals: check orthogonal directions iteratively
            # Only stop if detectForceNeighbor triggers (already handled)


    def prune_motions(self, node: Node):
        if not node.parent:
            return self.motions  # At start, all motions allowed

        px, py, pz = node.parent
        cx, cy, cz = node.current
        dx, dy, dz = cx - px, cy - py, cz - pz

        pruned = []
        for motion in self.motions:
            mx, my, mz = motion.current  # <-- unpack from Node.current
            # Keep motions that are along dx/dy/dz or diagonals
            if (dx == 0 or mx == dx or mx == 0) and \
            (dy == 0 or my == dy or my == 0) and \
            (dz == 0 or mz == dz or mz == 0):
                pruned.append(motion)
        return pruned

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

