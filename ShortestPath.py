import numpy as np
from python_tsp.exact import solve_tsp_dynamic_programming
from python_tsp.heuristics import solve_tsp_lin_kernighan
from python_tsp.heuristics import solve_tsp_record_to_record
from python_tsp.heuristics import solve_tsp_local_search
from python_tsp.exact import solve_tsp_brute_force
import time
class TSPSolver():
    def __init__(self, goalList, planner, startPose):
        goalList.insert(0, startPose)
        self.goalList = goalList
        self.planner = planner
        self.distance = None
        self.__solve()
    def __solve(self):
        start_time = time.time()
        graphSize = len(self.goalList)
        csGraph = np.zeros((graphSize,graphSize))
        i = 0
        j = 0
        for goal_i in self.goalList:
            for goal_j in self.goalList: 
                if goal_j == goal_i:
                    csGraph[i,j] = 0
                    break
                _, _, length = self.planner.solve(goal_i, goal_j, getPathLength=True, solveTime = 10)
                csGraph[i, j] = length
                j += 1
            j = 0
            i += 1
        csGraph = csGraph + csGraph.T
        print(f"CS Graph:\n{csGraph}\n\n\n")
        print("Solving TSP")
        permutation, self.distance = solve_tsp_local_search(csGraph)
        print("TSP Solutions: ", permutation)
        print("TSP SOLVE TIME: ", time.time() - start_time)
        print("TSP Distance: ", self.distance)
        self.goalList = [self.goalList[i] for i in permutation]
        self.goalList.append(self.goalList.pop(0))

    def getTSPPath(self):
        return self.goalList
