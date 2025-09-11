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
    # Create environment with boundary walls
    grid_env = Grid(40, 60, 12)
    
    # Add scattered trees throughout the countryside
    add_tree(grid_env, 8, 15, 5)
    add_tree(grid_env, 12, 8, 7)
    add_tree(grid_env, 15, 25, 6)
    add_tree(grid_env, 32, 18, 7)
    add_tree(grid_env, 35, 38, 5)
    add_tree(grid_env, 10, 42, 4)
    
    # Add a few rural buildings (barn, farmhouse)
    # Each unit = 10m, so 3x2x2 = 30x20x20m farmhouse
    add_building(grid_env, 5, 5, 3, 2, 2)     # Small farmhouse (30x20x20m)
    add_building(grid_env, 25, 8, 4, 3, 3)    # Barn (40x30x30m)
    add_building(grid_env, 15, 50, 2, 2, 2)   # Small shed (20x20x20m)
    
    return grid_env


if __name__ == '__main__':
    # Create countryside environment
    grid_env = create_countryside_environment()
    
    # Set start and goal points
    start = (3, 3, 1)
    goal = (35, 55, 1)
    
    # Choose and run a pathfinding algorithm
    # You can uncomment different algorithms to test them
    
    # Global planners
    plt = AStar(start, goal, env=grid_env)
    # plt = Dijkstra(start, goal, env=grid_env)
    # plt = JPS(start, goal, env=grid_env)
    # plt = ThetaStar(start, goal, env=grid_env) # 61 score
    # plt = DStar(start, goal, env=grid_env)
    # plt = DStarLite(start, goal, env=grid_env)
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
