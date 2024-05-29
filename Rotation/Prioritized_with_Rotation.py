from Single_Agent_Planner_with_Rotation import *
import time as timer

class PrioritizedPlanningSolver(object):
    """
    A planner that plans for each robot sequentially.
    """

    def __init__(self, my_map, starts, goals, max_time=None):
        """
        Initialize the Prioritized Planning Solver.
        
        Parameters:
        my_map (list): List of lists specifying obstacle positions.
        starts (list): List of start locations [(x1, y1), (x2, y2), ...].
        goals (list): List of goal locations [(x1, y1), (x2, y2), ...].
        max_time (int): Maximum time allowed for planning.
        """
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.max_time = max_time if max_time else float('inf')
        self.num_of_expanded = 0
        self.num_of_generated = 0
        self.CPU_time = 0

        # Compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """
        Finds paths for all agents from their start locations to their goal locations.
        
        Returns:
        list: List of paths for each agent.
        """
        start_time = timer.time()
        result = []
        constraints = []

        for i in range(self.num_of_agents):  # Finding the path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, constraints)
            print(f'Agent {i} path: {path}')
            if timer.time() - start_time >= self.max_time:
                raise BaseException("Time limit exceeded")
            if path is None:
                raise BaseException("No solution")

            result.append(path)

            for time, loc in enumerate(path):
                for a in range(self.num_of_agents):
                    if a != i:
                        # Vertex Constraint (other agents cannot be at `loc` at `time`)
                        constraints.append({
                            'agent': a,
                            'loc': [loc[0]],
                            'timestep': time,
                            'final': time == len(path) - 1
                        })
                        # Edge Constraint (other agents cannot move from `loc` to `next_loc` at `time + 1`)
                        if time < len(path) - 1:
                            next_loc = path[time + 1][0]
                            constraints.append({
                                'agent': a,
                                'loc': [loc[0], next_loc],
                                'timestep': time + 1,
                                'final': False
                            })
                        # Non-passing Constraint (agents cannot swap positions)
                        if time > 0:
                            prev_loc = path[time - 1][0]
                            constraints.append({
                                'agent': a,
                                'loc': [prev_loc, loc[0]],
                                'timestep': time,
                                'final': False
                            })
                            # Stronger non-passing Constraint (agents cannot pass each other at the same location)
                            constraints.append({
                                'agent': a,
                                'loc': [loc[0], prev_loc],
                                'timestep': time,
                                'final': False
                            })

        self.CPU_time = timer.time() - start_time
        return result