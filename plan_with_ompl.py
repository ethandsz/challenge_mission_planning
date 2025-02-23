

#!/usr/bin/env python3
import argparse
from OmplPlanner import OmplPlanner
from ShortestPath import TSPSolver
from scenarioHelpers import extractGoalsAndObstacles, getStartPose, read_scenario

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Single drone mission')

    parser.add_argument('scenario', type=str, help="scenario file to attempt to execute")
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
    tspSolver = TSPSolver(planner.goalPoints, planner, startPose)
    tspList = tspSolver.getTSPPath()
    f = open("trajectory.txt", "w")
    f.write("")
    start_point = getStartPose(scenario)
    for goal in tspList:
        goalPoint = [goal[0], goal[1], goal[2]]
        _, path = planner.solve(start_point, goalPoint)
        f = open("trajectory.txt", "a")
        f.write(path)
        f.close()
        start_point = goalPoint
    exit(0)
