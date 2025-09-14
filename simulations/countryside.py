"""
@file: countryside.py
@brief: Countryside simulation environment with mostly trees and natural obstacles
@author: Generated for IN5060 Assignment
@update: 2025.9.10
"""
import sys, os
import time
import argparse

from algopicker import algopicker
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning import *
from utils import add_building, add_tree


def create_countryside_environment():
    """Create a countryside environment with scattered trees and minimal buildings"""
    # Create environment with boundary walls (scaled 5x)
    grid_env = Grid(200, 300, 60)
    
    # Add scattered trees throughout the countryside (scaled 5x)
    add_tree(grid_env, 40, 75, 25)
    add_tree(grid_env, 60, 40, 35)
    add_tree(grid_env, 75, 125, 30)
    add_tree(grid_env, 160, 90, 35)
    add_tree(grid_env, 175, 190, 25)
    add_tree(grid_env, 50, 210, 20)
    
    # Add a few rural buildings (barn, farmhouse) (scaled 5x)
    # Each unit = 10m, so 15x10x10 = 150x100x100m farmhouse
    add_building(grid_env, 25, 25, 15, 10, 10)     # Small farmhouse (150x100x100m)
    add_building(grid_env, 125, 40, 20, 15, 15)    # Barn (200x150x150m)
    add_building(grid_env, 75, 250, 10, 10, 10)    # Small shed (100x100x100m)
    
    return grid_env


if __name__ == '__main__':
    # Create countryside environment
    grid_env = create_countryside_environment()
    
    # Set start and goal points (scaled 5x)
    start = (15, 15, 5)
    goal = (175, 275, 5)

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
    
    print("Countryside simulation completed!")
