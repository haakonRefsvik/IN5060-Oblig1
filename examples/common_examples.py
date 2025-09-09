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
    grid_env = Grid(20, 20, 20)
    # grid_env.obstacles already contains boundary walls by default
    
    # Add obstacles
    obstacles = grid_env.obstacles  # Get current obstacles (boundary walls)
    

    ## Vegg nr 1
    for x in range(5, 15, 1):
        for z in range(0, 20, 1):
            obstacles.add((x, 10, z))
    
    # Update the environment after adding obstacles:
    grid_env.update(obstacles)
    
    # map_env = Map(100, 100, 50)



    # -------------global planners-------------
    # plt = AStar(start=(1, 1, 18), goal=(18, 18, 1), env=grid_env) # Denne fungerer
    # plt = Dijkstra(start=(5, 5), goal=(45, 25), env=grid_env)
    
    # plt = DStar(start=(1, 1, 18), goal=(18, 18, 1), env=grid_env)  # Denne fungerer, men ser ikke utforskningsområde
    plt = DStarLite(start=(1, 1, 18), goal=(18, 18, 1), env=grid_env) # Denne fungerer
    # plt = ThetaStar(start=(1, 1, 18), goal=(18, 18, 1), env=grid_env) # Denne fungerer
    # plt = GBFS(start=(1, 1, 18), goal=(18, 18, 1), env=grid_env) # Denne funker, men ser ikke utforskningsområde
    # plt = JPS(start=(1, 1, 18), goal=(18, 18, 1), env=grid_env) # Denne fungerer

    
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