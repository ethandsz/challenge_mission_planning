

#!/usr/bin/env python3
import argparse
from OmplPlanner import OmplPlanner
from scenarioHelpers import extractGoalsAndObstacles, read_scenario

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
    planner = OmplPlanner(args.scenario)
    scenario = read_scenario(args.scenario)
    goalPoints, _ = extractGoalsAndObstacles(scenario)
    start_point = [0,0,1]
    f = open("trajectory.txt", "w")
    f.write("")
    for goal in goalPoints:
        goalPoint = [goal[0], goal[1], goal[2]]
        print(f"Going to {goalPoint}")
        _, path = planner.solve(start_point, goalPoint)
        f = open("trajectory.txt", "a")
        f.write(path)
        f.close()
        start_point = goalPoint
    exit(0)
