
#!/usr/bin/env python3
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting 
import matplotlib.pyplot as plt
import numpy as np
import argparse
from scenarioHelpers import extractGoalsAndObstacles, getStartPose, read_scenario



def plotMap(goalPoints, obstacles):
    ax = plt.figure().add_subplot(projection='3d')

    goalPoints = np.array(goalPoints)
    obstacles = np.array(obstacles)

    # Scatter all points at once
    ax.scatter(goalPoints[:, 0], goalPoints[:, 1], goalPoints[:, 2], c='black', label='goals')

    ax.scatter(obstacles[:, 3], obstacles[:, 4], obstacles[:, 5], c='red', label='obstacles')

    # Annotate goal points
    for i, g in enumerate(goalPoints):
        ax.text(g[0], g[1], g[2], str(i), color='black', fontsize=10)

    # Annotate obstacles
    for i, o in enumerate(obstacles):
        ax.text(o[3], o[4], o[5], str(i), color='red', fontsize=10)

    # Load and plot trajectory
    trajectory = np.loadtxt("trajectory.txt")
    xs = trajectory[:, 0]
    ys = trajectory[:, 1]
    zs = trajectory[:, 2]
     
    ax.plot(xs, ys, zs, 'b.-', label="Trajectory")


    ax.legend()
    ax.set_title(f"Trajectory For Scenario {args.scenario[-6]}")
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(0, 5)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()

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
    plotMap(goalPoints, obstacles)
    print('Clean exit')
    exit(0)
