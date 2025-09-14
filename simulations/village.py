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
    # Create environment with boundary walls (scaled 5x)
    grid_env = Grid(175, 225, 50)
    
    # Add village buildings (houses, shops, community buildings) (scaled 5x)
    # Each unit = 10m, so 10x10x10 = 100x100x100m house
    add_building(grid_env, 25, 40, 10, 10, 10)     # House 1 (100x100x100m)
    add_building(grid_env, 60, 30, 15, 10, 10)     # House 2 (150x100x100m)
    add_building(grid_env, 100, 40, 10, 10, 10)    # House 3 (100x100x100m)
    add_building(grid_env, 140, 50, 10, 10, 10)    # Small house (100x100x100m)
    
    add_building(grid_env, 40, 100, 15, 15, 15)    # Village shop (150x150x150m)
    add_building(grid_env, 90, 125, 20, 15, 15)    # Community center (200x150x150m)
    add_building(grid_env, 150, 150, 10, 10, 10)   # Another house (100x100x100m)
    
    add_building(grid_env, 25, 175, 15, 10, 10)    # House near edge (150x100x100m)
    add_building(grid_env, 75, 190, 10, 10, 10)    # Small cottage (100x100x100m)
    add_building(grid_env, 125, 200, 15, 10, 10)   # Larger house (150x100x100m)
    
    # Add trees for decoration around the village (scaled 5x)
    add_tree(grid_env, 15, 75, 30)
    add_tree(grid_env, 80, 75, 25)
    add_tree(grid_env, 110, 90, 35)
    add_tree(grid_env, 160, 100, 20)
    add_tree(grid_env, 50, 160, 40)
    add_tree(grid_env, 100, 175, 30)
    add_tree(grid_env, 150, 125, 25)
    add_tree(grid_env, 70, 210, 20)
    
    return grid_env


if __name__ == '__main__':
    # Create village environment
    grid_env = create_village_environment()
    
    # Set start and goal points (scaled 5x)
    start = (10, 10, 5)
    goal = (160, 210, 5)
    
    # Choose and run a pathfinding algorithm
    # You can uncomment different algorithms to test them
    
    # Global planners
    plt = algopicker()(start, goal, env=grid_env)
    
    # Time only the pathfinding computation
    start_time = time.time()
    cost, path, expand = plt.plan()
    end_time = time.time()
    
    # Calculate computation time
    execution_time = end_time - start_time
    
    # Create algorithm name with computation time
    algorithm_name = f"{str(plt)}. computation time: {execution_time:.4f}s"
    
    # Show the visualization with timing info (this part is not timed)
    plt.plot.animation(path, algorithm_name, cost, expand)
    
    print("Village simulation completed!")
