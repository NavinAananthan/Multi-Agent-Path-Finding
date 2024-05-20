from Single_agent_planner import *

grid_map = [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0]
    ]

goal = (2, 2)

h_table = compute_heuristics(grid_map,goal)

for key,value in h_table.items():
    print(f'{key}:{value}')

