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


def create_city_environment():
    """Create a dense city environment like Manhattan with tightly packed buildings"""
    # Create environment with boundary walls (scaled 5x)
    grid_env = Grid(250, 300, 125)
    
    # Create a dense Manhattan-like grid of buildings with wider streets for visibility
    # Row 1 - Southern edge buildings (scaled 5x)
    add_building(grid_env, 15, 15, 15, 20, 75)    # Skyscraper (150x200x750m)
    add_building(grid_env, 45, 15, 10, 20, 60)    # High-rise (100x200x600m)
    add_building(grid_env, 70, 15, 15, 20, 100)   # Skyscraper (150x200x1000m)
    add_building(grid_env, 100, 15, 10, 20, 90)   # Tall building (100x200x900m)
    add_building(grid_env, 125, 15, 15, 20, 40)   # Mid-rise (150x200x400m)
    add_building(grid_env, 155, 15, 10, 20, 110)  # Skyscraper (100x200x1100m)
    add_building(grid_env, 180, 15, 15, 20, 70)   # High-rise (150x200x700m)
    add_building(grid_env, 210, 15, 10, 20, 80)   # Tall building (100x200x800m)
    
    # Row 2 - Second street (y=50, scaled 5x)
    add_building(grid_env, 15, 50, 10, 15, 90)   # Narrow skyscraper (100x150x900m)
    add_building(grid_env, 40, 50, 15, 15, 125)  # Super tall (150x150x1250m)
    add_building(grid_env, 70, 50, 10, 15, 70)   # High-rise (100x150x700m)
    add_building(grid_env, 95, 50, 15, 15, 60)   # Wide building (150x150x600m)
    add_building(grid_env, 125, 50, 10, 15, 100) # Skyscraper (100x150x1000m)
    add_building(grid_env, 150, 50, 15, 15, 80)  # Tall building (150x150x800m)
    add_building(grid_env, 180, 50, 10, 15, 110) # Skyscraper (100x150x1100m)
    add_building(grid_env, 205, 50, 15, 15, 70)  # High-rise (150x150x700m)
    
    # Row 3 - Third street (y=85, scaled 5x)
    add_building(grid_env, 15, 85, 15, 20, 120)  # Skyscraper (150x200x1200m)
    add_building(grid_env, 45, 85, 10, 20, 55)   # Mid-rise (100x200x550m)
    add_building(grid_env, 70, 85, 15, 20, 95)   # Tall building (150x200x950m)
    add_building(grid_env, 100, 85, 10, 20, 75)  # High-rise (100x200x750m)
    add_building(grid_env, 125, 85, 15, 20, 105) # Skyscraper (150x200x1050m)
    add_building(grid_env, 155, 85, 10, 20, 65)  # Office (100x200x650m)
    add_building(grid_env, 180, 85, 15, 20, 85)  # Tall building (150x200x850m)
    add_building(grid_env, 210, 85, 10, 20, 115) # Skyscraper (100x200x1150m)
    
    # Row 4 - Fourth street (y=120, scaled 5x)
    add_building(grid_env, 15, 120, 10, 15, 80)   # High-rise (100x150x800m)
    add_building(grid_env, 40, 120, 15, 15, 100)  # Skyscraper (150x150x1000m)
    add_building(grid_env, 70, 120, 10, 15, 65)   # Office (100x150x650m)
    add_building(grid_env, 95, 120, 15, 15, 90)   # Wide building (150x150x900m)
    add_building(grid_env, 125, 120, 10, 15, 120) # Skyscraper (100x150x1200m)
    add_building(grid_env, 150, 120, 15, 15, 55)  # Mid-rise (150x150x550m)
    add_building(grid_env, 180, 120, 10, 15, 95)  # Tall building (100x150x950m)
    add_building(grid_env, 205, 120, 15, 15, 75)  # High-rise (150x150x750m)
    
    # Row 5 - Fifth street (y=155, scaled 5x)
    add_building(grid_env, 15, 155, 15, 20, 110)  # Skyscraper (150x200x1100m)
    add_building(grid_env, 45, 155, 10, 20, 70)   # High-rise (100x200x700m)
    add_building(grid_env, 70, 155, 15, 20, 80)   # Tall building (150x200x800m)
    add_building(grid_env, 100, 155, 10, 20, 100) # Skyscraper (100x200x1000m)
    add_building(grid_env, 125, 155, 15, 20, 50)  # Mid-rise (150x200x500m)
    add_building(grid_env, 155, 155, 10, 20, 90)  # Tall building (100x200x900m)
    add_building(grid_env, 180, 155, 15, 20, 115) # Skyscraper (150x200x1150m)
    add_building(grid_env, 210, 155, 10, 20, 60)  # Office (100x200x600m)
    
    # Row 6 - Sixth street (y=190, scaled 5x)
    add_building(grid_env, 15, 190, 10, 15, 85)   # High-rise (100x150x850m)
    add_building(grid_env, 40, 190, 15, 15, 105)  # Skyscraper (150x150x1050m)
    add_building(grid_env, 70, 190, 10, 15, 70)   # Office (100x150x700m)
    add_building(grid_env, 95, 190, 15, 15, 125)  # Super tall (150x150x1250m)
    add_building(grid_env, 125, 190, 10, 15, 55)  # Mid-rise (100x150x550m)
    add_building(grid_env, 150, 190, 15, 15, 90)  # Tall building (150x150x900m)
    add_building(grid_env, 180, 190, 10, 15, 110) # Skyscraper (100x150x1100m)
    add_building(grid_env, 205, 190, 15, 15, 65)  # Office (150x150x650m)
    
    # Row 7 - Seventh street (y=45)
    add_building(grid_env, 3, 45, 3, 4, 15)   # High-rise (30x40x150m)
    add_building(grid_env, 9, 45, 2, 4, 23)   # Skyscraper (20x40x230m)
    add_building(grid_env, 14, 45, 3, 4, 12)  # Wide building (30x40x120m)
    add_building(grid_env, 20, 45, 2, 4, 17)  # Tall building (20x40x170m)
    add_building(grid_env, 25, 45, 3, 4, 20)  # Skyscraper (30x40x200m)
    add_building(grid_env, 31, 45, 2, 4, 14)  # High-rise (20x40x140m)
    add_building(grid_env, 36, 45, 3, 4, 21)  # Skyscraper (30x40x210m)
    
    # Row 7 - Seventh street (y=225, scaled 5x)
    add_building(grid_env, 15, 225, 15, 20, 75)   # High-rise (150x200x750m)
    add_building(grid_env, 45, 225, 10, 20, 115)  # Skyscraper (100x200x1150m)
    add_building(grid_env, 70, 225, 15, 20, 60)   # Wide building (150x200x600m)
    add_building(grid_env, 100, 225, 10, 20, 85)  # Tall building (100x200x850m)
    add_building(grid_env, 125, 225, 15, 20, 100) # Skyscraper (150x200x1000m)
    add_building(grid_env, 155, 225, 10, 20, 70)  # High-rise (100x200x700m)
    add_building(grid_env, 180, 225, 15, 20, 105) # Skyscraper (150x200x1050m)
    add_building(grid_env, 210, 225, 10, 20, 55)  # Mid-rise (100x200x550m)
    
    # Row 8 - Eighth street (y=260, scaled 5x)
    add_building(grid_env, 15, 260, 10, 15, 95)   # Tall building (100x150x950m)
    add_building(grid_env, 40, 260, 15, 15, 65)   # Office (150x150x650m)
    add_building(grid_env, 70, 260, 10, 15, 110)  # Skyscraper (100x150x1100m)
    add_building(grid_env, 95, 260, 15, 15, 75)   # Wide building (150x150x750m)
    add_building(grid_env, 125, 260, 10, 15, 90)  # Tall building (100x150x900m)
    add_building(grid_env, 150, 260, 15, 15, 120) # Skyscraper (150x150x1200m)
    add_building(grid_env, 180, 260, 10, 15, 60)  # Office (100x150x600m)
    add_building(grid_env, 205, 260, 15, 15, 100) # Skyscraper (150x150x1000m)
    
    # Add minimal trees for small parks/plazas (scaled 5x)
    add_tree(grid_env, 60, 35, 20)   # Small park tree in street gap
    add_tree(grid_env, 115, 70, 25)  # Plaza tree in intersection
    add_tree(grid_env, 85, 105, 15)  # Street tree in wide area
    add_tree(grid_env, 170, 140, 20) # Park tree in open space
    add_tree(grid_env, 140, 175, 30) # Larger park tree in plaza
    add_tree(grid_env, 60, 210, 20)  # Plaza tree in intersection
    add_tree(grid_env, 195, 245, 25) # Street tree in northern area
    
    return grid_env


if __name__ == '__main__':
    # Create city environment
    grid_env = create_city_environment()
    
    # Set start and goal points for urban navigation (scaled 5x)
    start = (5, 5, 5)
    goal = (215, 280, 5)

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
    
    print("City simulation completed!")
