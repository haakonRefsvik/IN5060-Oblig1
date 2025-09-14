"""
@file: city.py
@brief: City simulation environment with dense buildings and urban layout
@author: Generated for IN5060 Assignment
@update: 2025.9.10
"""
import sys, os
import time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning import *
from utils import add_building, add_tree


def create_city_environment():
    """Create a dense city environment like Manhattan with tightly packed buildings"""
    # Create environment with boundary walls (scaled 10x)
    grid_env = Grid(500, 600, 250)
    
    # Create a dense Manhattan-like grid of buildings with wider streets for visibility
    # Row 1 - Southern edge buildings (wider spacing between buildings) (scaled 10x)
    add_building(grid_env, 30, 30, 30, 40, 150)    # Skyscraper (300x400x1500m)
    add_building(grid_env, 90, 30, 20, 40, 120)    # High-rise (200x400x1200m)
    add_building(grid_env, 140, 30, 30, 40, 200)   # Skyscraper (300x400x2000m)
    add_building(grid_env, 200, 30, 20, 40, 180)   # Tall building (200x400x1800m)
    add_building(grid_env, 250, 30, 30, 40, 80)    # Mid-rise (300x400x800m)
    add_building(grid_env, 310, 30, 20, 40, 220)   # Skyscraper (200x400x2200m)
    add_building(grid_env, 360, 30, 30, 40, 140)   # High-rise (300x400x1400m)
    add_building(grid_env, 420, 30, 20, 40, 160)   # Tall building (200x400x1600m)
    
    # Row 2 - Second street (y=100, wider gap from first row) (scaled 10x)
    add_building(grid_env, 30, 100, 20, 30, 180)   # Narrow skyscraper (200x300x1800m)
    add_building(grid_env, 80, 100, 30, 30, 250)   # Super tall (300x300x2500m)
    add_building(grid_env, 140, 100, 20, 30, 140)  # High-rise (200x300x1400m)
    add_building(grid_env, 190, 100, 30, 30, 120)  # Wide building (300x300x1200m)
    add_building(grid_env, 250, 100, 20, 30, 200)  # Skyscraper (200x300x2000m)
    add_building(grid_env, 300, 100, 30, 30, 160)  # Tall building (300x300x1600m)
    add_building(grid_env, 360, 100, 20, 30, 220)  # Skyscraper (200x300x2200m)
    add_building(grid_env, 410, 100, 30, 30, 140)  # High-rise (300x300x1400m)
    
    # Row 3 - Third street (y=170) (scaled 10x)
    add_building(grid_env, 30, 170, 30, 40, 240)   # Skyscraper (300x400x2400m)
    add_building(grid_env, 90, 170, 20, 40, 110)   # Mid-rise (200x400x1100m)
    add_building(grid_env, 140, 170, 30, 40, 190)  # Tall building (300x400x1900m)
    add_building(grid_env, 200, 170, 20, 40, 150)  # High-rise (200x400x1500m)
    add_building(grid_env, 250, 170, 30, 40, 210)  # Skyscraper (300x400x2100m)
    add_building(grid_env, 310, 170, 20, 40, 130)  # Office (200x400x1300m)
    add_building(grid_env, 360, 170, 30, 40, 170)  # Tall building (300x400x1700m)
    add_building(grid_env, 420, 170, 20, 40, 230)  # Skyscraper (200x400x2300m)
    
    # Row 4 - Fourth street (y=240) (scaled 10x)
    add_building(grid_env, 30, 240, 20, 30, 160)   # High-rise (200x300x1600m)
    add_building(grid_env, 80, 240, 30, 30, 200)   # Skyscraper (300x300x2000m)
    add_building(grid_env, 140, 240, 20, 30, 130)  # Office (200x300x1300m)
    add_building(grid_env, 190, 240, 30, 30, 180)  # Wide building (300x300x1800m)
    add_building(grid_env, 250, 240, 20, 30, 240)  # Skyscraper (200x300x2400m)
    add_building(grid_env, 300, 240, 30, 30, 110)  # Mid-rise (300x300x1100m)
    add_building(grid_env, 360, 240, 20, 30, 190)  # Tall building (200x300x1900m)
    add_building(grid_env, 410, 240, 30, 30, 150)  # High-rise (300x300x1500m)
    
    # Row 5 - Fifth street (y=310) (scaled 10x)
    add_building(grid_env, 30, 310, 30, 40, 220)   # Skyscraper (300x400x2200m)
    add_building(grid_env, 90, 310, 20, 40, 140)   # High-rise (200x400x1400m)
    add_building(grid_env, 140, 310, 30, 40, 160)  # Tall building (300x400x1600m)
    add_building(grid_env, 200, 310, 20, 40, 200)  # Skyscraper (200x400x2000m)
    add_building(grid_env, 250, 310, 30, 40, 100)  # Mid-rise (300x400x1000m)
    add_building(grid_env, 310, 310, 20, 40, 180)  # Tall building (200x400x1800m)
    add_building(grid_env, 360, 310, 30, 40, 230)  # Skyscraper (300x400x2300m)
    add_building(grid_env, 420, 310, 20, 40, 120)  # Office (200x400x1200m)
    
    # Row 6 - Sixth street (y=380) (scaled 10x)
    add_building(grid_env, 30, 380, 20, 30, 170)   # High-rise (200x300x1700m)
    add_building(grid_env, 80, 380, 30, 30, 210)   # Skyscraper (300x300x2100m)
    add_building(grid_env, 140, 380, 20, 30, 140)  # Office (200x300x1400m)
    add_building(grid_env, 190, 380, 30, 30, 250)  # Super tall (300x300x2500m)
    add_building(grid_env, 250, 380, 20, 30, 110)  # Mid-rise (200x300x1100m)
    add_building(grid_env, 300, 380, 30, 30, 180)  # Tall building (300x300x1800m)
    add_building(grid_env, 360, 380, 20, 30, 220)  # Skyscraper (200x300x2200m)
    add_building(grid_env, 410, 380, 30, 30, 130)  # Office (300x300x1300m)
    
    # Row 7 - Seventh street (y=450) (scaled 10x)
    add_building(grid_env, 30, 450, 30, 40, 150)   # High-rise (300x400x1500m)
    add_building(grid_env, 90, 450, 20, 40, 230)   # Skyscraper (200x400x2300m)
    add_building(grid_env, 140, 450, 30, 40, 120)  # Wide building (300x400x1200m)
    add_building(grid_env, 200, 450, 20, 40, 170)  # Tall building (200x400x1700m)
    add_building(grid_env, 250, 450, 30, 40, 200)  # Skyscraper (300x400x2000m)
    add_building(grid_env, 310, 450, 20, 40, 140)  # High-rise (200x400x1400m)
    add_building(grid_env, 360, 450, 30, 40, 210)  # Skyscraper (300x400x2100m)
    add_building(grid_env, 420, 450, 20, 40, 110)  # Mid-rise (200x400x1100m)
    
    # Row 8 - Eighth street (y=520) (scaled 10x)
    add_building(grid_env, 30, 520, 20, 30, 190)   # Tall building (200x300x1900m)
    add_building(grid_env, 80, 520, 30, 30, 130)   # Office (300x300x1300m)
    add_building(grid_env, 140, 520, 20, 30, 220)  # Skyscraper (200x300x2200m)
    add_building(grid_env, 190, 520, 30, 30, 150)  # Wide building (300x300x1500m)
    add_building(grid_env, 250, 520, 20, 30, 180)  # Tall building (200x300x1800m)
    add_building(grid_env, 300, 520, 30, 30, 240)  # Skyscraper (300x300x2400m)
    add_building(grid_env, 360, 520, 20, 30, 120)  # Office (200x300x1200m)
    add_building(grid_env, 410, 520, 30, 30, 200)  # Skyscraper (300x300x2000m)
    
    # Add minimal trees for small parks/plazas (placed in wider street intersections) (scaled 10x)
    add_tree(grid_env, 120, 70, 40)   # Small park tree in street gap
    add_tree(grid_env, 230, 140, 50)  # Plaza tree in intersection
    add_tree(grid_env, 170, 210, 30)  # Street tree in wide area
    add_tree(grid_env, 340, 280, 40)  # Park tree in open space
    add_tree(grid_env, 280, 350, 60)  # Larger park tree in plaza
    add_tree(grid_env, 120, 420, 40)  # Plaza tree in intersection
    add_tree(grid_env, 390, 490, 50)  # Street tree in northern area
    
    return grid_env


if __name__ == '__main__':
    # Create city environment
    grid_env = create_city_environment()
    
    # Set start and goal points for urban navigation (scaled 10x)
    start = (10, 10, 10)
    goal = (430, 560, 10)
    
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
    
    print("City simulation completed!")
