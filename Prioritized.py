from Single_agent_planner import *
import time as timer

class PrioritizedPlanningSolver(object):
     
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals, max_time=None):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.max_time = max_time if max_time else float('inf')
        self.num_of_expanded = 0
        self.num_of_generated = 0
        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))
        
    
    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []

        
        for i in range(self.num_of_agents): # Finding the path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, constraints)
            if timer.time()-start_time>=self.max_time:
                raise BaseException("Time limit Exceeded")
            if path is None:
                raise BaseException("No solution")
            
            for time,loc in enumerate(path):
                # create a new constraint with the current path location for all agents except the current one
                for a in range(self.num_of_agents):
                    if a!=i:
                        # Vertex Constraint eg(Other agents cannot be at (1, 1) at time 0)
                        constraints.append({
                            'agent': a,
                            'loc': [loc],
                            'timestep': time,
                            'final': time == len(path) - 1
                        })
                        # Edge Constarint eg(Other agents cannot move from (1, 1) to (1, 2) at time 1)
                        if time < len(path) - 1:
                            next_loc = path[path.index(loc)+1]
                            constraints.append({
                                'agent':a,
                                'loc': [next_loc,loc],
                                'timestep': time+1,
                                'final': False
                            })

        self.CPU_time = timer.time() - start_time

