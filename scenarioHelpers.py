import ast
import yaml

def extractGoalsAndObstacles(scenario: dict):
    """
    Run the mission for a single drone.

    :param drone_interface: DroneInterface object
    :return: Bool indicating if the mission was successful
    """
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

def getStartPose(scenario: dict):
    start_pose = list(scenario["drone_start_pose"].values())
    start_pose[-1] = 1.0 
    start_pose.append(0.0)
    return (start_pose)


def read_solution(file_path):
    file_path = file_path[:-5]
    solution_path = "solutions/" + file_path + ".txt"
    print(solution_path)
    try:
        with open(solution_path, 'r') as file:
            solution = file.read()
            print("Solution is: ", solution)
            solution = ast.literal_eval(solution)
            map(int, solution)
            return solution
    except:
        return None

def write_solution(file_path, permutation):
    file_path = file_path[:-5]
    solution_path = "solutions/" + file_path + ".txt"
    f = open(solution_path, "w")
    f.write(str(permutation))
    f.close()
