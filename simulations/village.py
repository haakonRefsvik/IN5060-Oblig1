"""
@file: village.py
@brief: Village simulation environment with small buildings and trees
@author: Generated for IN5060 Assignment
@update: 2025.9.10
"""
import sys, os
import time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning import *
from utils import add_building, add_tree


def create_village_environment():
    """Create a village environment with small buildings and decorative trees"""
    # Create environment with boundary walls (scaled 10x)
    grid_env = Grid(350, 450, 100)
    
    # Add village buildings (houses, shops, community buildings) (scaled 10x)
    # Each unit = 10m, so 20x20x20 = 200x200x200m house
    add_building(grid_env, 50, 80, 20, 20, 20)     # House 1 (200x200x200m)
    add_building(grid_env, 120, 60, 30, 20, 20)    # House 2 (300x200x200m)
    add_building(grid_env, 200, 80, 20, 20, 20)    # House 3 (200x200x200m)
    add_building(grid_env, 280, 100, 20, 20, 20)   # Small house (200x200x200m)
    
    add_building(grid_env, 80, 200, 30, 30, 30)    # Village shop (300x300x300m)
    add_building(grid_env, 180, 250, 40, 30, 30)   # Community center (400x300x300m)
    add_building(grid_env, 300, 300, 20, 20, 20)   # Another house (200x200x200m)
    
    add_building(grid_env, 50, 350, 30, 20, 20)    # House near edge (300x200x200m)
    add_building(grid_env, 150, 380, 20, 20, 20)   # Small cottage (200x200x200m)
    add_building(grid_env, 250, 400, 30, 20, 20)   # Larger house (300x200x200m)
    
    # Add trees for decoration around the village (scaled 10x)
    add_tree(grid_env, 30, 150, 60)
    add_tree(grid_env, 160, 150, 50)
    add_tree(grid_env, 220, 180, 70)
    add_tree(grid_env, 320, 200, 40)
    add_tree(grid_env, 100, 320, 80)
    add_tree(grid_env, 200, 350, 60)
    add_tree(grid_env, 300, 250, 50)
    add_tree(grid_env, 140, 420, 40)
    
    return grid_env


if __name__ == '__main__':
    # Create village environment
    grid_env = create_village_environment()
    
    # Set start and goal points (scaled 10x)
    start = (20, 20, 10)
    goal = (320, 420, 10)
    
    # Choose and run a pathfinding algorithm
    # You can uncomment different algorithms to test them
    
    # Global planners
    # plt = AStar(start, goal, env=grid_env)
    # plt = Dijkstra(start, goal, env=grid_env)
    # plt = JPS(start, goal, env=grid_env)
    plt = GBFS(start, goal, env=grid_env)
    
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
