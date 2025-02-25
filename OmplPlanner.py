#!/usr/bin/env python3
import numpy as np
from ompl import base as ob
from ompl import geometric as og
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock
import math
from queue import Queue

class OmplPlanner():
    def __init__(self, goalPoints, obstacles):
        self.nav_path = Path()
        self.ompl_path = None 
        self.obstacleBounds = np.empty((0,6))
        self.goalPoints, self.obstacles = goalPoints, obstacles
        
        self.totalPoints = len(self.goalPoints) + 1
        self.navPathQueue = Queue(maxsize=self.totalPoints) #add one for back to start
        print("Total Points to visit: ", self.totalPoints)
        self.createBoundsAroundGoalsAndObstacles()

    def createBoundsAroundGoalsAndObstacles(self, margin = 0.5, margin_corner = 0.1):
        goal_corners = []
        for goal in self.goalPoints:
            goal_x = goal[0] + 1.0 * math.cos(goal[3])
            goal_y = goal[1] + 1.0 * math.sin(goal[3])

            c_x, c_y, c_z = goal_x, goal_y, goal[2]
            width, depth, height = 0.05, 0.5, 0.25

            # Compute half-extents
            half_w = width / 2.0
            half_d = depth / 2.0
            half_h = height / 2.0

            # Define the four corners in the obstacle's local (unrotated) frame
            local_corners = np.array([
                [ half_w,  half_d],
                [ half_w, -half_d],
                [-half_w,  half_d],
                [-half_w, -half_d]
            ])
            yaw = goal[3]
            # Define the rotation matrix for yaw
            R = np.array([
                [np.cos(yaw), -np.sin(yaw)],
                [np.sin(yaw),  np.cos(yaw)]
            ])

            # Rotate local corners and shift by the center
            global_corners = np.dot(local_corners, R.T)
            global_corners[:, 0] += c_x
            global_corners[:, 1] += c_y

            # Determine the axis-aligned bounds in x and y
            x_min = np.min(global_corners[:, 0])
            x_max = np.max(global_corners[:, 0])
            y_min = np.min(global_corners[:, 1])
            y_max = np.max(global_corners[:, 1])

            # For z, rotation doesn't affect the bounds.
            z_min = c_z - half_h
            z_max = c_z + half_h

# Define all eight corners in the local (unrotated) frame
            local_corners_3d = np.array([
                [ half_w,  half_d,  half_h],
                [ half_w, -half_d,  half_h],
                # [-half_w,  half_d,  half_h],
                # [-half_w, -half_d,  half_h],
                [ half_w,  half_d, -half_h],
                [ half_w, -half_d, -half_h],
                # [-half_w,  half_d, -half_h],
                # [-half_w, -half_d, -half_h]
            ])

# Apply yaw rotation to x and y
            global_corners_3d = np.dot(local_corners_3d[:, :2], R.T)
            global_corners_3d = np.hstack([global_corners_3d, local_corners_3d[:, 2:]])  # Keep z unchanged

# Shift by the center position
            global_corners_3d[:, 0] += c_x
            global_corners_3d[:, 1] += c_y
            global_corners_3d[:, 2] += c_z

# Add each corner to the world config
            for i, (x, y, z) in enumerate(global_corners_3d):
                goal_corner = [0.9, 0.4, 0.9, x, y, z]
                goal_corners.append(goal_corner)

        for goal_corner in goal_corners:
            depth = goal_corner[0]
            height = goal_corner[1]
            width = goal_corner[2]
            obs_x = goal_corner[3]
            obs_y = goal_corner[4]
            obs_z = goal_corner[5]

            z_low = obs_z - (height/2)
            z_high = obs_z + (height/2)

            x_low = obs_x - (width/2)
            x_high = obs_x + (width/2)

            y_low = obs_y - (depth/2)
            y_high = obs_y + (depth/2)

            # Inflate the obstacle boundaries by the margin
            x_low_inflated = x_low #- margin_corner
            x_high_inflated = x_high #+ margin_corner
            y_low_inflated = y_low# - margin_corner
            y_high_inflated = y_high #+ margin_corner
            z_low_inflated = z_low #- margin_corner
            z_high_inflated = z_high #+ margin_corner
            print("Appending goal corner: ", [[x_low_inflated, x_high_inflated, y_low_inflated, y_high_inflated, z_low_inflated, z_high_inflated]])
            self.obstacleBounds = np.append(
                self.obstacleBounds,
                [[x_low_inflated, x_high_inflated, y_low_inflated, y_high_inflated, z_low_inflated, z_high_inflated]],
                axis=0
            )


        for obstacle in self.obstacles:
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
         
            self.obstacleBounds = np.append(
                self.obstacleBounds,
                [[x_low_inflated, x_high_inflated, y_low_inflated, y_high_inflated, z_low_inflated, z_high_inflated]],
                axis=0
            )

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

    def is_state_valid(self, state, obstacles, goals, margin = 1.0, goal_margin = 0.50):
        x = state.getX()
        y = state.getY()
        z = state.getZ()

        inside_any_obstacle = np.any(
            (self.obstacleBounds[:, 0] <= x) & (x <= self.obstacleBounds[:, 1]) &
            (self.obstacleBounds[:, 2] <= y) & (y <= self.obstacleBounds[:, 3]) &
            (self.obstacleBounds[:, 4] <= z) & (z <= self.obstacleBounds[:, 5])
        )
        if inside_any_obstacle:
            return False
        
        # for goal in goals:
        #     goal_x = goal[0] + 1.0 * math.cos(goal[3])
        #     goal_y = goal[1] + 1.0 * math.sin(goal[3])
        #
        #     c_x, c_y, c_z = goal_x, goal_y, goal[2]
        #     # width, depth, height = 0.9, 0.9, 0.7
        #     width, depth, height = 0.5, 0.5, 0.7
        #
        #     # Compute half-extents
        #     half_w = width / 2.0
        #     half_d = depth / 2.0
        #     half_h = height / 2.0
        #
        #     # Define the four corners in the obstacle's local (unrotated) frame
        #     local_corners = np.array([
        #         [ half_w,  half_d],
        #         [ half_w, -half_d],
        #         [-half_w,  half_d],
        #         [-half_w, -half_d]
        #     ])
        #     yaw = goal[3]
        #     # Define the rotation matrix for yaw
        #     R = np.array([
        #         [np.cos(yaw), -np.sin(yaw)],
        #         [np.sin(yaw),  np.cos(yaw)]
        #     ])
        #
        #     # Rotate local corners and shift by the center
        #     global_corners = np.dot(local_corners, R.T)
        #     global_corners[:, 0] += c_x
        #     global_corners[:, 1] += c_y
        #
        #     # Determine the axis-aligned bounds in x and y
        #     x_min = np.min(global_corners[:, 0])
        #     x_max = np.max(global_corners[:, 0])
        #     y_min = np.min(global_corners[:, 1])
        #     y_max = np.max(global_corners[:, 1])
        #
        #     # For z, rotation doesn't affect the bounds.
        #     z_min = c_z - half_h
        #     z_max = c_z + half_h
        #     if (x_min <= x <= x_max and
        #         y_min <= y <= y_max and
        #         z_min <= z <= z_max):
        #         return False
        return True

    def solveAll(self, start_position, goalPoints):
        print(goalPoints)
        for goal in goalPoints:
            reached_goal = False
            replans = 0
            solveTime = 5
            while not reached_goal:
                simplify_path = True
                if replans > 1:
                    simplify_path = False
                    solveTime = 30
                    print("No path simplification, replans are over 5")
                navpath, _ = self.solve(start_position, goal, solveTime=solveTime, simplify_path=simplify_path)
                last_pose = navpath.poses[-1]
                last_x = last_pose.pose.position.x 
                last_y = last_pose.pose.position.y 
                last_z = last_pose.pose.position.z
                last_position = [last_x, last_y, last_z]
                print(f"CHECKING {last_position} against {goal[:3]}")
                if last_position != goal[:3]:
                    print("REPLANNING")
                    replans += 1
                else:
                    reached_goal = True
                    self.navPathQueue.put([navpath, goal[3]])
                    start_position = [goal[0], goal[1], goal[2]]

    def getNextPath(self):
        return self.navPathQueue.get()

    def isTourDone(self):
        if self.totalPoints == 0:
            return True
    def is_path_valid(self) -> bool:
        return True
    def solve(self, start_position, goal_position, getPathLength = False, solveTime = 100, simplify_path = True):
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
            lambda state: self.is_state_valid(state, self.obstacleBounds, self.goalPoints)
        ))
        start_state = ob.State(space)

        start_state().setX(start_position[0])
        start_state().setY(start_position[1])
        start_state().setZ(start_position[2])
        start_state().rotation().setIdentity()

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
        
        planner = og.RRTConnect(ss.getSpaceInformation())
        # planner.setCostThreshold(0.1)
        # planner.setGoalBias(0.1)
        ss.setPlanner(planner)
        solutionsWindow = 50
        epsilon = 1e-5
        ccptc = ob.CostConvergenceTerminationCondition(ss.getProblemDefinition(),
                                                       solutionsWindow, epsilon)
        # plannerTerminationCondition = ob.PlannerTerminationCondition(lambda path: self.is_path_valid(path))
        # plannerTerminationCondition = ob.PlannerTerminationCondition(lambda: self.is_path_valid())
        # Solve within 5 seconds
        if ss.solve(solveTime):
            path = ss.getSolutionPath()
            if simplify_path:
                ss.simplifySolution(ccptc)  # Optional: Simplify the path
            path.interpolate(int(path.length()*1.5))  # Generate 100 waypoints
            navPath = self.path_to_navPath(path)
            if getPathLength:
                return navPath, path.printAsMatrix(), path.length()
            return navPath, path.printAsMatrix();
        raise Exception("No path found")

    def pathCompleted(self):
        self.totalPoints -= 1
