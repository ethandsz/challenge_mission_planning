#!/usr/bin/env python3
import numpy as np
from ompl import base as ob
from ompl import geometric as og
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock
from scenarioHelpers import extractGoalsAndObstacles, read_scenario
import math

class OmplPlanner():
    def __init__(self, scenario_file):
        self.scenario_file = scenario_file
        self.nav_path = Path()
        self.ompl_path = None 
    def getPath(self):
        if self.nav_path.length == 0:
            raise ValueError("No Path Defined/Found") 
        else:
            return self.nav_path

    def path_to_navPath(self, path, frame_id="world"):
        """
        Convert an OMPL geometric path (an instance of ompl.geometric.PathGeometric)
        to a ROS nav_msgs/Path message.

        Parameters:
          ompl_path: the OMPL path (assumed to be in SE(3))
          frame_id: the frame in which the path is defined (default "world")
        
        Returns:
          A nav_msgs.msg.Path message containing the poses from the OMPL path.
        """
        nav_path = Path()
        nav_path.header.frame_id = frame_id
        clock = Clock()
        nav_path.header.stamp = clock.now().to_msg()
        # Retrieve the list of states in the OMPL path.
        # Each state is assumed to be an SE(3) state.
        # print(f"Iterating states {states.length}")
        for i in range(path.getStateCount()):


            # IMPORTANT: Call the state to get the underlying object.
            s = path.getState(i)
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.header.stamp = clock.now().to_msg()  # or use nav_path.header.stamp

            # Extract position from the state.
            ps.pose.position.x = s.getX()
            ps.pose.position.y = s.getY()
            ps.pose.position.z = s.getZ()

            # Extract orientation from the state.
            rot = s.rotation()
            ps.pose.orientation.x = rot.x
            ps.pose.orientation.y = rot.y
            ps.pose.orientation.z = rot.z
            ps.pose.orientation.w = rot.w

            nav_path.poses.append(ps)

        return nav_path

    def is_state_valid(self, state, obstacles, goals, margin = 1.0, goal_margin = 0.55):
        x = state.getX()
        y = state.getY()
        z = state.getZ()
        for obstacle in obstacles:
            depth = obstacle[0]
            height = obstacle[1]
            width = obstacle[2]
            obs_x = obstacle[3]
            obs_y = obstacle[4]
            obs_z = obstacle[5]

            z_low = obs_z - (height/2)
            z_high = obs_z + (height/2)

            x_low = obs_x - (width/2)
            x_high = obs_x + (width/2)

            y_low = obs_y - (depth/2)
            y_high = obs_y + (depth/2)

            # Inflate the obstacle boundaries by the margin
            x_low_inflated = x_low - margin
            x_high_inflated = x_high + margin
            y_low_inflated = y_low - margin
            y_high_inflated = y_high + margin
            z_low_inflated = z_low - margin
            z_high_inflated = z_high + margin

            # If the state is within the inflated boundaries, it violates the constraint
            if (x_low_inflated <= x <= x_high_inflated and
                y_low_inflated <= y <= y_high_inflated and
                z_low_inflated <= z <= z_high_inflated):
                return False

            

        for goal in goals:
            # Extract goal parameters
            goal_z = goal[2]
            goal_w = goal[3]
            
            # Compute the true goal center after applying an offset along the local x-axis
            goal_x = goal[0] + 1.0 * math.cos(goal_w)
            goal_y = goal[1] + 1.0 * math.sin(goal_w)
            
            true_goal = np.array([goal_x, goal_y, goal_z])
            state_pos = np.array([x, y, z])
            
            # Use Euclidean distance as a safety radius check
            distance = np.linalg.norm(state_pos - true_goal)
            if distance < goal_margin:
                return False
            
            
        return True

    def solve(self, start_position, goal_position):
        scenario = read_scenario(self.scenario_file)
        goalPoints, obstacles = extractGoalsAndObstacles(scenario)
        space = ob.SE3StateSpace()
        bounds = ob.RealVectorBounds(3)
        bounds.setLow(0, -15)
        bounds.setHigh(0, 15)
        bounds.setLow(1, -15)
        bounds.setHigh(1, 15)
        bounds.setLow(2, 0)
        bounds.setHigh(2, 7)
        space.setBounds(bounds)
        ss = og.SimpleSetup(space)
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(
            lambda state: self.is_state_valid(state, obstacles, goalPoints)
        ))
        start_state = ob.State(space)

        start_state().setX(start_position[0])
        start_state().setY(start_position[1])
        start_state().setZ(start_position[2])
        start_state().rotation().setIdentity()
        # ss.setStartState(start_state)

        goal_states = ob.GoalStates(ss.getSpaceInformation())

        goal_state = ob.State(space)
        goal_state().setX(goal_position[0])
        goal_state().setY(goal_position[1])
        goal_state().setZ(goal_position[2])
        goal_state().rotation().setIdentity()
        goal_states.addState(goal_state)

        # ss.setGoal(goal_states)
        pdef = ss.getProblemDefinition()
        pdef.setStartAndGoalStates(start_state, goal_state)

        pdef = ss.getProblemDefinition()
        ss.setStartAndGoalStates(start_state, goal_state)
        
        planner = og.RRTstar(ss.getSpaceInformation())
        ss.setPlanner(planner)
        # Solve within 5 seconds
        if ss.solve(15):
            ss.simplifySolution(10.0)  # Optional: Simplify the path
            path = ss.getSolutionPath()
            path.interpolate(10)  # Generate 100 waypoints
            navPath = self.path_to_navPath(path)
            return navPath, path.printAsMatrix();
        raise Exception("No path found")



