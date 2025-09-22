"""
@file: utils.py
@brief: Utility functions for creating obstacles in 3D grid environments
@author: Generated for IN5060 Assignment
@update: 2025.9.10
"""
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning import *


def add_building(grid: Grid, x_offset, y_offset, length, width, height):
    # Add obstacles
    obstacles = grid.obstacles  # Get current obstacles (boundary walls)

    for y in range(0, width + 1, 1):
        for x in range(0, length + 1, 1):
            for z in range(0, height, 1):
                obstacles.add((x + x_offset, y + y_offset, z))

    # Update the environment after adding obstacles:
    grid.update(obstacles)



def add_tree(grid: Grid, x_offset, y_offset, height):
    # Add obstacles
    obstacles = grid.obstacles  # Get current obstacles (boundary walls)
    leafoffsets = [[1, 0], [-1, 0], [0, 1], [0, -1]]

    for z in range(0, height, 1):
        obstacles.add((x_offset, y_offset, z))

    for offset in leafoffsets:
        for z in range(2, height - 1, 1):
            obstacles.add((x_offset + offset[0], y_offset + offset[1], z))
        
    # Update the environment after adding obstacles:
    grid.update(obstacles)


def add_obstacle_block(grid: Grid, x_offset, y_offset, z_offset, length, width, height):
    """Add a solid block of obstacles at specified position"""
    obstacles = grid.obstacles
    
    for x in range(length):
        for y in range(width):
            for z in range(height):
                obstacles.add((x + x_offset, y + y_offset, z + z_offset))
    
    grid.update(obstacles)


def clear_path(grid: Grid, start_pos, end_pos, width=1):
    """Clear a path between two points (useful for creating roads/walkways)"""
    # This is a simple implementation - you could make it more sophisticated
    obstacles = grid.obstacles
    x1, y1, z1 = start_pos
    x2, y2, z2 = end_pos
    
    # Simple linear interpolation to clear path
    steps = max(abs(x2-x1), abs(y2-y1))
    if steps == 0:
        return
        
    for i in range(steps + 1):
        t = i / steps
        x = int(x1 + t * (x2 - x1))
        y = int(y1 + t * (y2 - y1))
        z = int(z1 + t * (z2 - z1))
        
        # Clear area around the path
        for dx in range(-width//2, width//2 + 1):
            for dy in range(-width//2, width//2 + 1):
                pos = (x + dx, y + dy, z)
                if pos in obstacles:
                    obstacles.remove(pos)
    
    grid.update(obstacles)
    
