

#!/usr/bin/env python3
import numpy as np
import argparse
from time import sleep
import time
import yaml
from ompl import base as ob
from ompl import geometric as og

def is_state_valid(state, obstacles):
    return True

def extractGoalsAndObstacles(scenario: dict):
    """
    Run the mission for a single drone.

    :param drone_interface: DroneInterface object
    :return: Bool indicating if the mission was successful
    """
    print('Run mission')
    goalPoints = [] 
    obstacles = []
    # Go to path facing
    for vpid, vp in scenario["viewpoint_poses"].items():
        print(f'Goal {vpid}, with path facing {vp}')
        goal = [vp["x"], vp["y"], vp["z"]]
        goalPoints.append(goal)

    for obid, ob in scenario["obstacles"].items():
        print(f'Obstacle {obid}, {ob}')
        obstacle = [ob["d"],ob["h"],ob["w"],ob["x"],ob["y"],ob["z"]]
        obstacles.append(obstacle)

    return goalPoints, obstacles

# Read the YAML scenario file
def read_scenario(file_path):
    with open(file_path, 'r') as file:
        scenario = yaml.safe_load(file)
    return scenario

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


    print(f"Reading scenario {args.scenario}")
    scenario = read_scenario(args.scenario)
    goalPoints, obstacles = extractGoalsAndObstacles(scenario)
    space = ob.SE3StateSpace()
    bounds = ob.RealVectorBounds(3)
    bounds.setLow(0, -10)
    bounds.setHigh(0, 10)
    bounds.setLow(1, -10)
    bounds.setHigh(1, 10)
    bounds.setLow(2, 0)
    bounds.setHigh(2, 5)
    space.setBounds(bounds)
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(
        lambda state: is_state_valid(state, obstacles)
    ))
    print(space.getBounds().low[2]) 
    start_state = ob.State(space)
    start_state().setX(0)
    start_state().setY(0)
    start_state().setZ(0)
    start_state().rotation().setIdentity()
    print("Is start valid?", is_state_valid(start_state(), obstacles))
    ss.setStartState(start_state)

    goal_states = ob.GoalStates(ss.getSpaceInformation())

    for goal in goalPoints:
        goal_state = ob.State(space)
        goal_state().setX(goal[0])
        goal_state().setY(goal[1])
        goal_state().setZ(goal[2])
        goal_state().rotation().setIdentity()
        goal_states.addState(goal_state)

    ss.setGoal(goal_states)

    planner = og.RRTstar(ss.getSpaceInformation()).setup()
    ss.setPlanner(planner)
    # Solve within 5 seconds
    if ss.solve(5.0):
        ss.simplifySolution()  # Optional: Simplify the path
        path = ss.getSolutionPath()
        path.interpolate(1000)  # Generate 100 waypoints
        print(path.printAsMatrix())
        f = open("trajectory.txt", "w")
        f.write(path.printAsMatrix())
        f.close()

    print('Clean exit')
    exit(0)



