
import argparse
from python_motion_planning import *
from python_motion_planning.global_planner.graph_search.graph_search import GraphSearcher

def algopicker() -> GraphSearcher:
    parser = argparse.ArgumentParser(description="Countryside motion planning simulation")
    parser.add_argument(
        "algorithm",
        type=str,
        choices=["astar", "dijkstra", "jps", "gbfs"],
        help="Which global planner to run"
    )
    args = parser.parse_args()

    algorithms = {
        "astar": AStar,
        "dijkstra": Dijkstra,
        "jps": JPS,
        "thetastar": ThetaStar,
        "dstar": DStar,
        "dstarlite": DStarLite,
        "gbfs": GBFS,
    }

    pickedAlgo = algorithms[args.algorithm]

    return pickedAlgo