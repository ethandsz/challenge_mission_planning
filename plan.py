
#!/usr/bin/env python3
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting 
import matplotlib.pyplot as plt
import numpy as np
import argparse
from scenarioHelpers import extractGoalsAndObstacles, read_scenario


def plotMap(goalPoints, obstacles):

    ax = plt.figure().add_subplot(projection='3d')
    # for g in goalPoints:
    #     ax.scatter(g[0], g[1], g[2], c='black', label='goal')
    goalPoints = np.array(goalPoints)
    obstacles = np.array(obstacles)
    # Scatter all points at once
    ax.scatter(goalPoints[:, 0], goalPoints[:, 1], goalPoints[:, 2], c='black', label='goals')

    ax.scatter(obstacles[:, 3], obstacles[:, 4], obstacles[:, 5], c='red', label='obstacles')
# Plot obstacles as cylinders
    trajectory = np.loadtxt("trajectory.txt")
    xs = trajectory[:, 0]
    ys = trajectory[:, 1]
    zs = trajectory[:, 2]
     
    ax.plot(xs, ys, zs, 'b.-', label="Trajectory")
    ax.legend()
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(0, 5)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
# Rotate the axes and update
    for angle in range(0, 360 + 1):
        # Normalize the angle to the range [-180, 180] for display
        angle_norm = (angle + 180) % 360 - 180

        # Cycle through a full rotation of elevation, then azimuth, roll, and all
        elev = azim = roll = 0
        if angle <= 360:
            azim = angle_norm

        # Update the axis view and title
        ax.view_init(elev, azim, roll)
        plt.title('Elevation: %d°, Azimuth: %d°, Roll: %d°' % (elev, azim, roll))

        plt.draw()
        plt.pause(.001)
        # ax.view_init(elev=10., azim=-35, roll=0)
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
