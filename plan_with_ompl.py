

#!/usr/bin/env python3
import argparse
from OmplPlanner import OmplPlanner
from ShortestPath import TSPSolver
from scenarioHelpers import extractGoalsAndObstacles, getStartPose, read_scenario,read_solution
def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Single drone mission')

    parser.add_argument('scenario', type=str, help="scenario file to attempt to execute")

    parser.add_argument('useTSPSolver', type=str2bool, nargs='?', default=True, help="Use the TSP Solver (True/False)")
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        default=False,
                        help='Enable verbose output')

    args = parser.parse_args()
    verbosity = args.verbose
    scenario = read_scenario(args.scenario)
    goalPoints, obstacles = extractGoalsAndObstacles(scenario)
    startPose = getStartPose(scenario)
    planner = OmplPlanner(goalPoints, obstacles)

    solution_path = read_solution(args.scenario)
    print(args.useTSPSolver)
    print("READ SOLUTION")
    if args.useTSPSolver:
        if solution_path is not None:
            print(f"Previous solution found:\n{solution_path}")
            goalList = solution_path 
        else:
            print("No solution found, please hold caller...")
            tspSolver = TSPSolver(planner.goalPoints,planner, startPose)
            goalList = tspSolver.getTSPPath()
    else:
        goalList = goalPoints
    f = open("trajectory.txt", "w")
    f.write("")
    start_point = getStartPose(scenario)
    total_path_len = 0
    for goal in goalList:
        goalPoint = [goal[0], goal[1], goal[2]]
        _, path, path_len = planner.solve(start_point, goalPoint, getPathLength=True)
        total_path_len += path_len
        f = open("trajectory.txt", "a")
        f.write(path)
        f.close()
        start_point = goalPoint
    print(f"PATH LENGTH IS {total_path_len} FOR {args.scenario}")
    exit(0)
