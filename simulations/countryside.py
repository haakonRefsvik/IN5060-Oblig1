"""
@file: countryside.py
@brief: Countryside simulation environment with mostly trees and natural obstacles
@author: Generated for IN5060 Assignment
@update: 2025.9.10
"""
import sys, os
import time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning import *
from utils import add_building, add_tree


def create_countryside_environment():
    """Create a countryside environment with scattered trees and minimal buildings"""
    # Create environment with boundary walls (scaled 10x)
    grid_env = Grid(400, 600, 120)
    
    # Add scattered trees throughout the countryside (scaled 10x)
    add_tree(grid_env, 80, 150, 50)
    add_tree(grid_env, 120, 80, 70)
    add_tree(grid_env, 150, 250, 60)
    add_tree(grid_env, 320, 180, 70)
    add_tree(grid_env, 350, 380, 50)
    add_tree(grid_env, 100, 420, 40)
    
    # Add a few rural buildings (barn, farmhouse) (scaled 10x)
    # Each unit = 10m, so 30x20x20 = 300x200x200m farmhouse
    add_building(grid_env, 50, 50, 30, 20, 20)     # Small farmhouse (300x200x200m)
    add_building(grid_env, 250, 80, 40, 30, 30)    # Barn (400x300x300m)
    add_building(grid_env, 150, 500, 20, 20, 20)   # Small shed (200x200x200m)
    
    return grid_env


if __name__ == '__main__':
    # Create countryside environment
    grid_env = create_countryside_environment()
    
    # Set start and goal points (scaled 10x)
    start = (30, 30, 10)
    goal = (350, 550, 10)
    
    # Choose and run a pathfinding algorithm
    # You can uncomment different algorithms to test them
    
    # Global planners
    # plt = AStar(start, goal, env=grid_env)
    # plt = Dijkstra(start, goal, env=grid_env)
    plt = JPS(start, goal, env=grid_env)
    # plt = GBFS(start, goal, env=grid_env)
    
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
