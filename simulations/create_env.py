

import random

from utils import add_building, add_tree, add_obstacle_block 
from python_motion_planning import *

def create_env(
    x: int,
    y: int,
    z: int,
    building_density: float = 0.0012,
    tree_density: float = 0.001
):
    """
    Create a countryside environment with random rural buildings and trees as obstacles.

    Parameters
    ----------
    x, y, z : int
        Dimensions of the grid.
    building_density : float, optional
        Approx. number of buildings per cell (default ~0.0012 ≈ 3 buildings in 50×50 grid).
    tree_density : float, optional
        Approx. number of trees per cell (default ~0.001 ≈ 2–3 trees in 50×50 grid).

    Returns
    -------
    Grid
        A Grid object populated with buildings and trees, both treated as obstacles.
    """
    grid_env = Grid(x, y, z)

    # --- Random Trees as obstacles ---
    # Calculate how many trees to place based on density
    tree_count = max(1, round(tree_density * x * y))
    for _ in range(tree_count):
        tx = random.randint(0, x - 1)
        ty = random.randint(0, y - 1)
        tz = random.randint(3, 7)  # variable tree height, adjust as needed
        add_tree(grid_env, tx, ty, tz)  # add_tree should mark these cells as occupied

    # --- Random Buildings as obstacles ---
    building_count = max(1, round(building_density * x * y))
    for _ in range(building_count):
        bw = random.randint(2, 4)  # width in 10m units
        bd = random.randint(2, 3)  # depth
        bh = random.randint(2, 15)  # height

        bx = random.randint(0, max(0, x - bw - 1))
        by = random.randint(0, max(0, y - bd - 1))

        add_building(grid_env, bx, by, bw, bd, bh)

    return grid_env
