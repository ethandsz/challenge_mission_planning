
import yaml

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
        goal = [vp["x"], vp["y"], vp["z"], vp["w"]]
        goalPoints.append(goal)

    for obid, ob in scenario["obstacles"].items():
        obstacle = [ob["d"],ob["h"],ob["w"],ob["x"],ob["y"],ob["z"]]
        obstacles.append(obstacle)

    return goalPoints, obstacles

# Read the YAML scenario file
def read_scenario(file_path):
    with open(file_path, 'r') as file:
        scenario = yaml.safe_load(file)
    return scenario
