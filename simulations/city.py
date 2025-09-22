"""
@file: city.py
@brief: City simulation environment with dense buildings and urban layout
@author: Generated for IN5060 Assignment
@update: 2025.9.10
"""
import sys, os
import time

from algopicker import algopicker
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning import *
from utils import add_building, add_tree


def create_city_environment(x, y, z = 25):
    """Create a dense city environment like Manhattan with tightly packed buildings"""
    # Create environment with boundary walls
    grid_env = Grid(x, y, z)
    
    # Create a dense Manhattan-like grid of buildings with wider streets for visibility
    # Row 1 - Southern edge buildings (wider spacing between buildings)
    add_building(grid_env, 3, 3, 3, 4, 15)    # Skyscraper (30x40x150m)
    add_building(grid_env, 9, 3, 2, 4, 12)    # High-rise (20x40x120m)
    add_building(grid_env, 14, 3, 3, 4, 20)   # Skyscraper (30x40x200m)
    add_building(grid_env, 20, 3, 2, 4, 18)   # Tall building (20x40x180m)
    add_building(grid_env, 25, 3, 3, 4, 8)    # Mid-rise (30x40x80m)
    add_building(grid_env, 31, 3, 2, 4, 22)   # Skyscraper (20x40x220m)
    add_building(grid_env, 36, 3, 3, 4, 14)   # High-rise (30x40x140m)
    add_building(grid_env, 42, 3, 2, 4, 16)   # Tall building (20x40x160m)
    
    # Row 2 - Second street (y=10, wider gap from first row)
    add_building(grid_env, 3, 10, 2, 3, 18)   # Narrow skyscraper (20x30x180m)
    add_building(grid_env, 8, 10, 3, 3, 25)   # Super tall (30x30x250m)
    add_building(grid_env, 14, 10, 2, 3, 14)  # High-rise (20x30x140m)
    add_building(grid_env, 19, 10, 3, 3, 12)  # Wide building (30x30x120m)
    add_building(grid_env, 25, 10, 2, 3, 20)  # Skyscraper (20x30x200m)
    add_building(grid_env, 30, 10, 3, 3, 16)  # Tall building (30x30x160m)
    add_building(grid_env, 36, 10, 2, 3, 22)  # Skyscraper (20x30x220m)
    add_building(grid_env, 41, 10, 3, 3, 14)  # High-rise (30x30x140m)
    
    # Row 3 - Third street (y=17)
    add_building(grid_env, 3, 17, 3, 4, 24)   # Skyscraper (30x40x240m)
    add_building(grid_env, 9, 17, 2, 4, 11)   # Mid-rise (20x40x110m)
    add_building(grid_env, 14, 17, 3, 4, 19)  # Tall building (30x40x190m)
    add_building(grid_env, 20, 17, 2, 4, 15)  # High-rise (20x40x150m)
    add_building(grid_env, 25, 17, 3, 4, 21)  # Skyscraper (30x40x210m)
    add_building(grid_env, 31, 17, 2, 4, 13)  # Office (20x40x130m)
    add_building(grid_env, 36, 17, 3, 4, 17)  # Tall building (30x40x170m)
    add_building(grid_env, 42, 17, 2, 4, 23)  # Skyscraper (20x40x230m)
    
    # Row 4 - Fourth street (y=24)
    add_building(grid_env, 3, 24, 2, 3, 16)   # High-rise (20x30x160m)
    add_building(grid_env, 8, 24, 3, 3, 20)   # Skyscraper (30x30x200m)
    add_building(grid_env, 14, 24, 2, 3, 13)  # Office (20x30x130m)
    add_building(grid_env, 19, 24, 3, 3, 18)  # Wide building (30x30x180m)
    add_building(grid_env, 25, 24, 2, 3, 24)  # Skyscraper (20x30x240m)
    add_building(grid_env, 30, 24, 3, 3, 11)  # Mid-rise (30x30x110m)
    add_building(grid_env, 36, 24, 2, 3, 19)  # Tall building (20x30x190m)
    add_building(grid_env, 41, 24, 3, 3, 15)  # High-rise (30x30x150m)
    
    # Row 5 - Fifth street (y=31)
    add_building(grid_env, 3, 31, 3, 4, 22)   # Skyscraper (30x40x220m)
    add_building(grid_env, 9, 31, 2, 4, 14)   # High-rise (20x40x140m)
    add_building(grid_env, 14, 31, 3, 4, 16)  # Tall building (30x40x160m)
    add_building(grid_env, 20, 31, 2, 4, 20)  # Skyscraper (20x40x200m)
    add_building(grid_env, 25, 31, 3, 4, 10)  # Mid-rise (30x40x100m)
    add_building(grid_env, 31, 31, 2, 4, 18)  # Tall building (20x40x180m)
    add_building(grid_env, 36, 31, 3, 4, 23)  # Skyscraper (30x40x230m)
    add_building(grid_env, 42, 31, 2, 4, 12)  # Office (20x40x120m)
    
    # Row 6 - Sixth street (y=38)
    add_building(grid_env, 3, 38, 2, 3, 17)   # High-rise (20x30x170m)
    add_building(grid_env, 8, 38, 3, 3, 21)   # Skyscraper (30x30x210m)
    add_building(grid_env, 14, 38, 2, 3, 14)  # Office (20x30x140m)
    add_building(grid_env, 19, 38, 3, 3, 25)  # Super tall (30x30x250m)
    add_building(grid_env, 25, 38, 2, 3, 11)  # Mid-rise (20x30x110m)
    add_building(grid_env, 30, 38, 3, 3, 18)  # Tall building (30x30x180m)
    add_building(grid_env, 36, 38, 2, 3, 22)  # Skyscraper (20x30x220m)
    add_building(grid_env, 41, 38, 3, 3, 13)  # Office (30x30x130m)
    
    # Row 7 - Seventh street (y=45)
    add_building(grid_env, 3, 45, 3, 4, 15)   # High-rise (30x40x150m)
    add_building(grid_env, 9, 45, 2, 4, 23)   # Skyscraper (20x40x230m)
    add_building(grid_env, 14, 45, 3, 4, 12)  # Wide building (30x40x120m)
    add_building(grid_env, 20, 45, 2, 4, 17)  # Tall building (20x40x170m)
    add_building(grid_env, 25, 45, 3, 4, 20)  # Skyscraper (30x40x200m)
    add_building(grid_env, 31, 45, 2, 4, 14)  # High-rise (20x40x140m)
    add_building(grid_env, 36, 45, 3, 4, 21)  # Skyscraper (30x40x210m)
    add_building(grid_env, 42, 45, 2, 4, 11)  # Mid-rise (20x40x110m)
    
    # Row 8 - Eighth street (y=52) 
    add_building(grid_env, 3, 52, 2, 3, 19)   # Tall building (20x30x190m)
    add_building(grid_env, 8, 52, 3, 3, 13)   # Office (30x30x130m)
    add_building(grid_env, 14, 52, 2, 3, 22)  # Skyscraper (20x30x220m)
    add_building(grid_env, 19, 52, 3, 3, 15)  # Wide building (30x30x150m)
    add_building(grid_env, 25, 52, 2, 3, 18)  # Tall building (20x30x180m)
    add_building(grid_env, 30, 52, 3, 3, 24)  # Skyscraper (30x30x240m)
    add_building(grid_env, 36, 52, 2, 3, 12)  # Office (20x30x120m)
    add_building(grid_env, 41, 52, 3, 3, 20)  # Skyscraper (30x30x200m)
    
    # Add minimal trees for small parks/plazas (placed in wider street intersections)
    add_tree(grid_env, 12, 7, 4)   # Small park tree in street gap
    add_tree(grid_env, 23, 14, 5)  # Plaza tree in intersection
    add_tree(grid_env, 17, 21, 3)  # Street tree in wide area
    add_tree(grid_env, 34, 28, 4)  # Park tree in open space
    add_tree(grid_env, 28, 35, 6)  # Larger park tree in plaza
    add_tree(grid_env, 12, 42, 4)  # Plaza tree in intersection
    add_tree(grid_env, 39, 49, 5)  # Street tree in northern area
    
    return grid_env


if __name__ == '__main__':
    # Create city environment
    grid_env = create_city_environment(50, 60, 25)
    
    # Set start and goal points for urban navigation
    start = (1, 1, 1)
    goal = (43, 56, 1)

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
    # Calculate computation time
    
    # Create algorithm name with computation time
    algorithm_name = f"{str(plt)}. computation time: {execution_time:.4f}s"
    
    # Show the visualization with timing info (this part is not timed)
    plt.plot.animation(path, algorithm_name, cost, expand)
    
    print("City simulation completed!")
