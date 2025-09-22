"""
@file: countryside.py
@brief: Countryside simulation environment with mostly trees and natural obstacles
@author: Generated for IN5060 Assignment
@update: 2025.9.10
"""
import random
import sys, os
import time
import argparse

from algopicker import algopicker
from create_env import create_env
from python_motion_planning.global_planner.graph_search.graph_search import GraphSearcher
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning import *
from utils import add_building, add_tree


if __name__ == '__main__':
    # Create countryside environment
    z = 20
    #density = [0.0012, 0.001]
    #density = [0.0024, 0.002]
    density = [0.05, 0.0001]
    run_n_times = 1
    cost_runs = []
    ex_runs = []

    for x_and_y in range(50, 60, 10):
        grid_env: Grid = create_env(x_and_y, x_and_y, z, density[0], density[1])
        ex_times = []
        costs = []
        for i in range(run_n_times):
            while True:
                start = (random.randint(1, x_and_y-1),
                        random.randint(1, x_and_y-1),
                        1)
                if start not in grid_env.obstacles:
                    break

            while True:
                goal = (random.randint(1, x_and_y-1),
                        random.randint(1, x_and_y-1),
                        1)
                if goal not in grid_env.obstacles and goal != start:
                    break

            plt: GraphSearcher = algopicker()(start, goal, env=grid_env)
            start_time = time.time()
            cost, path, expand = plt.plan()
            end_time = time.time()
            execution_time = end_time - start_time

            c = f"{cost:.3f}"
            e = f"{execution_time:.3f}"
            costs.append(c)
            ex_times.append(e)
        cost_runs.append(costs)
        ex_runs.append(ex_times)
        print(f"Map x and y: {x_and_y}")    

    print("exec:")
    for l in ex_runs:
        print(l)
    print("cost")
    for l in cost_runs:
        print(l)
    
    # Create algorithm name with computation time
    algorithm_name = f"{str(plt)}. computation time: {execution_time:.4f}s"
    
    # Show the visualization with timing info (this part is not timed)
    plt.plot.animation(path, algorithm_name, cost, expand)
    
    print("Countryside simulation completed!")
