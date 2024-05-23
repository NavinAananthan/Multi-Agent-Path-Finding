from Prioritized import *


grid_map = [
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0]
    ]
    

starts = [(2, 0)]
goals = [(2, 4)]

PPS = PrioritizedPlanningSolver(grid_map,starts,goals)
print(PPS.find_solution())