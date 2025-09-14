"""
@file: village.py
@brief: Village simulation environment with small buildings and trees
@author: Generated for IN5060 Assignment
@update: 2025.9.10
"""
import sys, os
import time

from algopicker import algopicker
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning import *
from utils import add_building, add_tree


def create_village_environment():
    """Create a village environment with small buildings and decorative trees"""
    # Create environment with boundary walls
    grid_env = Grid(35, 45, 10)
    
    # Add village buildings (houses, shops, community buildings)
    # Each unit = 10m, so 2x3x2 = 20x30x20m house
    add_building(grid_env, 5, 8, 2, 2, 2)     # House 1 (20x20x20m)
    add_building(grid_env, 12, 6, 3, 2, 2)    # House 2 (30x20x20m)
    add_building(grid_env, 20, 8, 2, 2, 2)    # House 3 (20x20x20m)
    add_building(grid_env, 28, 10, 2, 2, 2)   # Small house (20x20x20m)
    
    add_building(grid_env, 8, 20, 3, 3, 3)    # Village shop (30x30x30m)
    add_building(grid_env, 18, 25, 4, 3, 3)   # Community center (40x30x30m)
    add_building(grid_env, 30, 30, 2, 2, 2)   # Another house (20x20x20m)
    
    add_building(grid_env, 5, 35, 3, 2, 2)    # House near edge (30x20x20m)
    add_building(grid_env, 15, 38, 2, 2, 2)   # Small cottage (20x20x20m)
    add_building(grid_env, 25, 40, 3, 2, 2)   # Larger house (30x20x20m)
    
    # Add trees for decoration around the village
    add_tree(grid_env, 3, 15, 6)
    add_tree(grid_env, 16, 15, 5)
    add_tree(grid_env, 22, 18, 7)
    add_tree(grid_env, 32, 20, 4)
    add_tree(grid_env, 10, 32, 8)
    add_tree(grid_env, 20, 35, 6)
    add_tree(grid_env, 30, 25, 5)
    add_tree(grid_env, 14, 42, 4)
    
    return grid_env


if __name__ == '__main__':
    # Create village environment
    grid_env = create_village_environment()
    
    # Set start and goal points
    start = (2, 2, 1)
    goal = (32, 42, 1)
    
    # Choose and run a pathfinding algorithm
    # You can uncomment different algorithms to test them
    
    # Global planners
    plt = algopicker()(start, goal, env=grid_env)
    
    # Time only the pathfinding computation
    run_n_times = 1
    ex_times = []
    for i in range(run_n_times):
        start_time = time.time()
        cost, path, expand = plt.plan()
        end_time = time.time()
        execution_time = end_time - start_time
        e = f"{execution_time:.3f} s"
        ex_times.append(e)
    
    print(ex_times)

    # Create algorithm name with computation time
    algorithm_name = f"{str(plt)}. computation time: {execution_time:.4f}s"
    
    # Show the visualization with timing info (this part is not timed)
    plt.plot.animation(path, algorithm_name, cost, expand)
    
    print("Village simulation completed!")
