"""
@file: common_examples.py
@breif: Examples of Python Motion Planning library
@author: Wu Maojia
@update: 2025.4.11
"""
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning import *

if __name__ == '__main__':
    # Create environment with no custom obstacles (only boundary walls)
    grid_env = Grid(100, 100, 50)
    # grid_env.obstacles already contains boundary walls by default
    
    # Add obstacles
    obstacles = grid_env.obstacles  # Get current obstacles (boundary walls)
    
    # Add a 3D cube obstacle in the middle:
    center_x, center_y, center_z = 50, 25, 25
    cube_size = 10
    for x in range(center_x - cube_size, center_x + cube_size + 1):
        for y in range(center_y - cube_size, center_y + cube_size + 1):
            for z in range(center_z - cube_size, center_z + cube_size + 1):
                if 0 <= x < 100 and 0 <= y < 100 and 0 <= z < 50:
                    obstacles.add((x, y, z))
    
    # Update the environment after adding obstacles:
    grid_env.update(obstacles)
    
    # map_env = Map(100, 100, 50)



    # -------------global planners-------------
    plt = AStar(start=(5, 5, 45), goal=(95, 95, 5), env=grid_env)
    # plt = DStar(start=(5, 5, 45), goal=(95, 95, 5), env=grid_env)
    # plt = DStarLite(start=(5, 5), goal=(45, 25), env=grid_env)
    # plt = Dijkstra(start=(5, 5), goal=(45, 25), env=grid_env)
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