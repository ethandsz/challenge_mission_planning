#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Simple mission for a single drone."""

__authors__ = 'Rafael Perez-Segui'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'
from threading import Thread
import argparse
from time import sleep
import time
import yaml
from ShortestPath import TSPSolver
from mission_camera import DroneMotionRef
from OmplPlanner import OmplPlanner
from as2_python_api.drone_interface import DroneInterface
import rclpy
from scenarioHelpers import extractGoalsAndObstacles, getStartPose, read_scenario, read_solution, write_solution
TAKE_OFF_HEIGHT = 1.0  # Height in meters
TAKE_OFF_SPEED = 1.0  # Max speed in m/s
SLEEP_TIME = 1.0  # Sleep time between behaviors in seconds
SPEED = 1.0  # Max speed in m/s
LAND_SPEED = 0.5  # Max speed in m/s

def drone_start(drone_interface: DroneInterface) -> bool:
    """
    Take off the drone.

    :param drone_interface: DroneInterface object
    :return: Bool indicating if the take off was successful
    """
    print('Start mission')

    # Arm
    print('Arm')
    success = drone_interface.arm()
    print(f'Arm success: {success}')

    # Offboard
    print('Offboard')
    success = drone_interface.offboard()
    print(f'Offboard success: {success}')

    # Take Off
    print('Take Off')
    success = drone_interface.takeoff(height=TAKE_OFF_HEIGHT, speed=TAKE_OFF_SPEED)
    print(f'Take Off success: {success}')

    return success


def drone_run(drone_interface: DroneInterface, timing: dict) -> bool:
    """
    Run the mission for a single drone.

    :param drone_interface: DroneInterface object
    :return: Bool indicating if the mission was successful
    """
    print('Run mission')
    
    scenario = read_scenario(args.scenario)
    goalsPoints, obstacles = extractGoalsAndObstacles(scenario)
    startPose = getStartPose(scenario) 
    omplPlanner = OmplPlanner(goalsPoints, obstacles)
    solution_path = read_solution(args.scenario)
    if solution_path is not None:
        print(f"Previous solution found, reading from: {solution_path}")
        tspList = solution_path 
    else:
        print("No solution found, please hold caller...")
        tspSolver = TSPSolver(omplPlanner.goalPoints,omplPlanner, startPose)
        tspList = tspSolver.getTSPPath()
        write_solution(args.scenario, tspList)
    thread = Thread(target=omplPlanner.solveAll, args=(drone_interface.position, tspList,))
    thread.start()
    started = False
    while not omplPlanner.isTourDone():
        path = omplPlanner.getNextPath()
        if path is not None: 
            if(not started):
                started = True
                timing['start_time'] = time.time()
            success = drone_interface.follow_path.follow_path_with_yaw(path[0], speed = 5.0, angle = path[1])
            if not success:
                return success
            print("Finished a goal")
            omplPlanner.pathCompleted()
            sleep(SLEEP_TIME)
    return True;

def drone_end(drone_interface: DroneInterface) -> bool:
    """
    End the mission for a single drone.

    :param drone_interface: DroneInterface object
    :return: Bool indicating if the land was successful
    """
    print('End mission')

    # Land
    print('Land')
    success = drone_interface.land(speed=LAND_SPEED)
    print(f'Land success: {success}')
    if not success:
        return success

    # Manual
    print('Manual')
    success = drone_interface.manual()
    print(f'Manual success: {success}')

    return success


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Single drone mission')

    parser.add_argument('scenario', type=str, help="scenario file to attempt to execute")
    parser.add_argument('-n', '--namespace',
                        type=str,
                        default='drone0',
                        help='ID of the drone to be used in the mission')
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        default=False,
                        help='Enable verbose output')
    parser.add_argument('-s', '--use_sim_time',
                        action='store_true',
                        default=True,
                        help='Use simulation time')

    args = parser.parse_args()
    drone_namespace = args.namespace
    verbosity = args.verbose
    use_sim_time = args.use_sim_time

    print(f'Running mission for drone {drone_namespace}')

    print(f"Reading scenario {args.scenario}")
    scenario = read_scenario(args.scenario)
    
    rclpy.init()

    uav = DroneInterface(
        drone_id=drone_namespace,
        use_sim_time=use_sim_time,
        verbose=verbosity)

    success = drone_start(uav)
    try:
        timing = {'start_time': None}  # Create a dictionary to store the start_time
        if success:
            success = drone_run(uav, timing)
        if timing['start_time'] is not None:
            duration = time.time() - timing['start_time'] 
            print("---------------------------------")
            print(f"Tour of {args.scenario} took {duration} seconds")
            print("---------------------------------")
    except KeyboardInterrupt as e:
        pass
    # sleep(5.0)
    success = drone_end(uav)

    uav.shutdown()
    rclpy.shutdown()
    print('Clean exit')
    exit(0)
