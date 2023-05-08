#!/bin/bash

usage() {
    echo "usage: $0 [-p <ign_gz|dji_osdk>] [-r] [-t] [drone_namespace]"
    echo ""
    echo "  options:"
    echo "      -b: launch behavior tree"
    echo "      -m: multi agent, choices: [true | false]"
    echo "      -r: record rosbag"
    echo "      -t: launch keyboard teleoperation"
    echo "      drone_namespace: [drone_sim_0 | drone_0]"
}

# Arg parser
while getopts "bwrt" opt; do
  echo "Command line arguments: $@"
  case ${opt} in
    b )
      behavior_tree="true"
      ;;
    m )
      swarm="true"
      ;;
    r )
      record_rosbag="true"
      ;;
    t )
      launch_keyboard_teleop="true"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[wrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

echo ${swarm}
# Shift optional args
shift $((OPTIND -1))

## DEFAULTS
behavior_tree=${behavior_tree:="false"}
swarm=${swarm:="false"}
record_rosbag=${record_rosbag:="false"}
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}

if [[ ${swarm} == "true" ]]; then
  simulation_config="sim_config/world_swarm.json"
  num_drones=3
else
  simulation_config="sim_config/world.json" 
  num_drones=1
fi

# Generate the list of drone namespaces
drone_ns=()
for ((i=0; i<num_drones; i++)); do
  drone_ns+=("cf$i")
done

for ns in "${drone_ns[@]}"
do
  if [[ ${ns} == ${drone_ns[0]} ]]; then
    base_launch="true"
  else
    base_launch="false"
  fi 

  tmuxinator start -n ${ns} -p utils/session.yml drone_namespace=${ns} estimator_plugin="ground_truth" behavior_tree=${behavior_tree} record_rosbag=${record_rosbag} launch_keyboard_teleop=${launch_keyboard_teleop} simulation_config=${simulation_config} &
  wait

done

tmuxinator start -n gazebo -p utils/gazebo.yml simulation_config=${simulation_config} &
wait

# Attach to tmux session ${drone_ns[@]}, window 0
tmux attach-session -t ${drone_ns[0]}:mission

pkill -9 -f "gazebo"
