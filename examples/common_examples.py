"""
@file: common_examples.py
@breif: Examples of Python Motion Planning library
@author: Wu Maojia
@update: 2025.4.11
"""
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning import *


def add_building(grid: Grid, x_offset, y_offset, length, width, height):
    # Add obstacles
    obstacles = grid.obstacles  # Get current obstacles (boundary walls)

    for x in range(0, length + 1, 1):
        for z in range(0, height, 1):
            obstacles.add((x + x_offset, y_offset, z))
    
    for x in range(0, length + 1, 1):
        for z in range(0, height, 1):
            obstacles.add((x + x_offset, y_offset + width, z))

    for y in range(0, width, 1):
        for z in range(0, height, 1):
            obstacles.add((x_offset, y + y_offset, z))

    for y in range(0, width, 1):
        for z in range(0, height, 1):
            obstacles.add((x_offset + length, y + y_offset, z))

    ## roof
    for y in range(0, width + 1, 1):
        for x in range(0, length + 1, 1):
            obstacles.add((x + x_offset, y + y_offset, height))

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

if __name__ == '__main__':
    # Create environment with no custom obstacles (only boundary walls)
    grid_env = Grid(30, 50, 15)
    add_building(grid_env, 5, 5, 4, 4, 8)       # ifi
    add_building(grid_env, 20, 40, 7, 5, 4)     # bunnpris
    add_building(grid_env, 18, 30, 7, 5, 10)     # høyt bygg

    add_tree(grid_env, 10, 20, 4)

    start = (7, 4, 1)
    goal = (22, 39, 1)

    # -------------global planners-------------


    
    # plt = AStar(start, goal, env=grid_env)
    plt = Dijkstra(start, goal, env=grid_env)
    #plt = DStar(start=(1, 1, 10), goal=(15, 15, 5), env=grid_env)
    #plt = DStarLite(start=(5, 9, 6), goal=(25, 25, 5), env=grid_env)
    # plt = GBFS(start=(5, 5), goal=(45, 25), env=grid_env)
    # plt = JPS(start=(5, 5), goal=(45, 25), env=grid_env)
    # plt = ThetaStar(start=(5, 5), goal=(45, 25), env=grid_env)

    # plt = LazyThetaStar(start=(5, 5), goal=(45, 25), env=grid_env)
    # plt = SThetaStar(start=(5, 5), goal=(45, 25), env=grid_env)
    # plt = LPAStar(start=(5, 5), goal=(45, 25), env=grid_env)
    # plt = VoronoiPlanner(start=(5, 5), goal=(45, 25), env=grid_env)

    # plt = RRT(start=(18, 8), goal=(37, 18), env=map_env)
    # plt = RRTConnect(start=(18, 8), goal=(37, 18), env=map_env)
    # plt = RRTStar(start=(18, 8), goal=(37, 18), env=map_env)
    # plt = InformedRRT(start=(18, 8), goal=(37, 18), env=map_env)

    # plt = ACO(start=(5, 5), goal=(45, 25), env=grid_env)
    # plt = PSO(start=(5, 5), goal=(45, 25), env=grid_env)

    plt.run()

    # -------------local planners-------------
    # plt = PID(start=(5, 5, 0), goal=(45, 25, 0), env=grid_env)
    # plt = DWA(start=(5, 5, 0), goal=(45, 25, 0), env=grid_env)
    # plt = APF(start=(5, 5, 0), goal=(45, 25, 0), env=grid_env)
    # plt = LQR(start=(5, 5, 0), goal=(45, 25, 0), env=grid_env)
    # plt = RPP(start=(5, 5, 0), goal=(45, 25, 0), env=grid_env)
    # plt = MPC(start=(5, 5, 0), goal=(45, 25, 0), env=grid_env)
    # plt.run()

    # -------------curve generators-------------
    # points = [(0, 0, 0), (10, 10, -90), (20, 5, 60), (30, 10, 120),
    #           (35, -5, 30), (25, -10, -120), (15, -15, 100), (0, -10, -90)]

    # plt = Dubins(step=0.1, max_curv=0.25)
    # plt = Bezier(step=0.1, offset=3.0)
    # plt = Polynomial(step=2, max_acc=1.0, max_jerk=0.5)
    # plt = ReedsShepp(step=0.1, max_curv=0.25)
    # plt = CubicSpline(step=0.1)
    # plt = BSpline(step=0.01, k=3)

    # plt.run(points)