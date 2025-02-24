import yaml
import numpy as np
import json
import math
import argparse
import os
from jinja2 import Template

# Template for SDF file
SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{{ model_name }}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box>
            <size>{{ width }} {{ depth }} {{ height }}</size>
          </box>
        </geometry>
        <material>
          <ambient>1.0 0.5 0.5 1.0</ambient>
          <diffuse>1.0 0.5 0.5 1.0</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>{{ width }} {{ depth }} {{ height }}</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""

# Template for model.config
MODEL_CONFIG_TEMPLATE = """<?xml version="1.0" ?>
<model>
  <name>{{ model_name }}</name>
  <version>1.0</version>
  <sdf version="1.6">{{ model_name }}.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  <description>
    A cuboid model with dimensions: Width={{ width }}, Depth={{ depth }}, Height={{ height }}
  </description>
</model>
"""

def generate_cuboid_model(output_dir, model_name, width, depth, height):
    """Generate SDF and model.config for a cuboid."""
    model_dir = os.path.join(output_dir, model_name)
    os.makedirs(model_dir, exist_ok=True)
    
    # Render SDF file
    sdf_content = Template(SDF_TEMPLATE).render(
        model_name=model_name,
        width=width,
        depth=depth,
        height=height
    )
    with open(os.path.join(model_dir, f"{model_name}.sdf"), "w") as sdf_file:
        sdf_file.write(sdf_content)
    
    # Render model.config
    config_content = Template(MODEL_CONFIG_TEMPLATE).render(
        model_name=model_name,
        width=width,
        depth=depth,
        height=height
    )
    with open(os.path.join(model_dir, "model.config"), "w") as config_file:
        config_file.write(config_content)

# Function to calculate marker pose
# Marker is placed 1 meter in front of the viewpoint's orientation (yaw)
def calculate_marker_pose(viewpoint, marker_distance=1.0):
    x = viewpoint['x'] + marker_distance * math.cos(viewpoint['w'])
    y = viewpoint['y'] + marker_distance * math.sin(viewpoint['w'])
    z = viewpoint['z']  # Assuming marker is at the same height as the viewpoint
    print(f"X: {x}, Y: {y}, Z: {z}")
    return {'x': x, 'y': y, 'z': z}

# Read the YAML scenario file
def read_scenario(file_path):
    with open(file_path, 'r') as file:
        scenario = yaml.safe_load(file)
    return scenario

def getStartPose(scenario: dict):
    start_pose = list(scenario["drone_start_pose"].values())
    return (start_pose)
# Write the JSON world configuration
def write_world_config(scenario, model_type, world_name, output_folder, world_file_name, marker_distance):
    os.makedirs(output_folder, exist_ok=True)
    start_pose = getStartPose(scenario)
    start_pose[-1] = 0.5
    world_config = {
        "world_name": world_name,
        "origin": {
            "latitude": 40.4405287,
            "longitude": -3.6898277,
            "altitude": 100.0
        },
        "drones": [
            {
                "model_type": model_type,
                "model_name": "drone0",
                "xyz": start_pose, 
                "rpy": [0, 0, 0.0],
                "flight_time": 60,
                "payload": [
                    {
                        "model_name": "hd_camera",
                        "model_type": "hd_camera",
                        "rpy": [0.0, 0.0, 0.0]
                    }
                ]
            }
        ],
        "objects": []
    }

    # Add obstacles as objects
    for key, obstacle in scenario.get('obstacles', {}).items():
        model_name = f"obstacle_{key}"
        world_config['objects'].append({
            "model_type": model_name,
            "model_name": model_name,
            "xyz": [obstacle['x'], obstacle['y'], obstacle['z']],
            "rpy": [0, 0, 0]
        })
        generate_cuboid_model(
            os.path.join(output_folder, "models"), model_name, 
            obstacle['w'], obstacle['d'], obstacle['h'])

    # Add viewpoint markers as ArUco markers
    # As in config_sim/gazebo/models all aruco models are idX4_marker
    for key, viewpoint in scenario.get('viewpoint_poses', {}).items():
        marker_pose = calculate_marker_pose(viewpoint, marker_distance)
        model_type = f"aruco_id{(key%8) + 1}4_marker"
        world_config['objects'].append({
            "model_type": model_type,
            "model_name": f"id{key}",
            "xyz": [marker_pose['x'], marker_pose['y'], marker_pose['z']],
            "rpy": [0, math.pi/2.0, viewpoint['w'] - math.pi]
        })
#         goal = [viewpoint['x'], viewpoint['y'], viewpoint['z'], viewpoint['w']]
#
#         goal_x = goal[0] + 1.0 * math.cos(goal[3])
#         goal_y = goal[1] + 1.0 * math.sin(goal[3])
#
#         c_x, c_y, c_z = goal_x, goal_y, goal[2]
#         width, depth, height = 0.05, 0.5, 0.5
#         # width, depth, height = 0.2, 0.2, 0.2
#
#         # Compute half-extents
#         half_w = width / 2.0
#         half_d = depth / 2.0
#         half_h = height / 2.0
#
#         # Define the four corners in the obstacle's local (unrotated) frame
#         local_corners = np.array([
#             [ half_w,  half_d],
#             [ half_w, -half_d],
#             [-half_w,  half_d],
#             [-half_w, -half_d]
#         ])
#         yaw = goal[3]
#         # Define the rotation matrix for yaw
#         R = np.array([
#             [np.cos(yaw), -np.sin(yaw)],
#             [np.sin(yaw),  np.cos(yaw)]
#         ])
#
#         # Rotate local corners and shift by the center
#         global_corners = np.dot(local_corners, R.T)
#         global_corners[:, 0] += c_x
#         global_corners[:, 1] += c_y
#
#         # Determine the axis-aligned bounds in x and y
#         x_min = np.min(global_corners[:, 0])
#         x_max = np.max(global_corners[:, 0])
#         y_min = np.min(global_corners[:, 1])
#         y_max = np.max(global_corners[:, 1])
#
#         # For z, rotation doesn't affect the bounds.
#         z_min = c_z - half_h
#         z_max = c_z + half_h
#
#         print("XMIN: ", x_min)
#         print("YMIN: ", y_min)
#         print("ZMIN: ", z_min)
#
#         model_name_center = f"obstacle_{key}_center"
#         world_config['objects'].append({
#             "model_type": model_name_center,
#             "model_name": model_name_center,
#             "xyz": [float(c_x), float(c_y), float(c_z)],
#             "rpy": [0, 0, 0]
#         })
#         generate_cuboid_model(
#             os.path.join(output_folder, "models"), model_name_center, 
#             0.1, 0.1, 0.1)
#         corner_names = ["corner_1", "corner_2", "corner_3", "corner_4", "corner_5", "corner_6", "corner_7", "corner_8"]
#
# # Define all eight corners in the local (unrotated) frame
#         local_corners_3d = np.array([
#             [ half_w,  half_d,  half_h],
#             [ half_w, -half_d,  half_h],
#             # [-half_w,  half_d,  half_h],
#             # [-half_w, -half_d,  half_h],
#             [ half_w,  half_d, -half_h],
#             [ half_w, -half_d, -half_h],
#             # [-half_w,  half_d, -half_h],
#             # [-half_w, -half_d, -half_h]
#         ])
#
# # Apply yaw rotation to x and y
#         global_corners_3d = np.dot(local_corners_3d[:, :2], R.T)
#         global_corners_3d = np.hstack([global_corners_3d, local_corners_3d[:, 2:]])  # Keep z unchanged
#
# # Shift by the center position
#         global_corners_3d[:, 0] += c_x
#         global_corners_3d[:, 1] += c_y
#         global_corners_3d[:, 2] += c_z
#
# # Add each corner to the world config
#         for i, (x, y, z) in enumerate(global_corners_3d):
#             model_name = f"obstacle_{key}_{corner_names[i]}"
#             world_config['objects'].append({
#                 "model_type": model_name,
#                 "model_name": model_name,
#                 "xyz": [float(x), float(y), float(z)],
#                 "rpy": [0, 0, 0]
#             })
#             generate_cuboid_model(
#                 os.path.join(output_folder, "models"), model_name, 
#                 0.6, 0.6, 0.2
#             )
    with open(os.path.join(output_folder, world_file_name), 'w') as file:
        yaml.dump(world_config, file)
        # json.dump(world_config, file, indent=4)

# Main function
def main():
    parser = argparse.ArgumentParser(description="Generate JSON world configuration from YAML scenario.")
    parser.add_argument('input_file', type=str, help="Path to the input YAML scenario file.")
    parser.add_argument('output_folder', type=str, help="Folder to the output YAML world configuration file and generated models")
    parser.add_argument('--model_type', type=str, default="quadrotor_base", help="Model type for the drone.")
    parser.add_argument('--world_name', type=str, default="empty", help="Name of the world.")
    parser.add_argument('--world_file_name', type=str, default="world.yaml", help="Name of the world.")
    parser.add_argument('--marker_distance', type=float, default=1.0, help="Distance away from viewpoint to generate marker.")

    args = parser.parse_args()

    scenario = read_scenario(args.input_file)
    write_world_config(scenario, args.model_type, args.world_name, args.output_folder, args.world_file_name, args.marker_distance)
    print(f"Scenario from {args.input_file} world configuration written to {args.output_folder}")

if __name__ == "__main__":
    main()
