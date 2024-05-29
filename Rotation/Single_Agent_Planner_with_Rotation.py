import heapq



def move(loc, direction):
    """
    Calculate the new location by moving in the given direction.
    
    Parameters:
    loc (tuple): Current location (x, y).
    direction (int): Direction index (0: North, 1: East, 2: South, 3: West).
    
    Returns:
    tuple: New location after the move.
    """
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    return loc[0] + directions[direction][0], loc[1] + directions[direction][1]


def flatten_constraints(constraints):
    """
    Flatten the list of constraints.
    
    Parameters:
    constraints (list): List of constraints.
    
    Returns:
    list: Flattened list of constraints.
    """
    flat_constraints = []
    for constr_list in constraints:
        for c in constr_list:
            flat_constraints.append(c)
    return flat_constraints


def get_path(goal_node):
    """
    Retrieve the path from the start to the goal node.
    
    Parameters:
    goal_node (dict): Goal node in the search tree.
    
    Returns:
    list: Path from start to goal.
    """
    path = []
    curr = goal_node
    while curr is not None:
        path.append((curr['loc'], curr['time']))
        curr = curr['parent']
    path.reverse()
    return path


def moveRotation(loc, dir, currDir):
    """
    Compute the movement steps considering rotations.
    
    Parameters:
    loc (tuple): Current location (x, y).
    dir (int): Target direction index (0: North, 1: East, 2: South, 3: West, 4: Wait).
    currDir (int): Current direction index.
    
    Returns:
    list: List of steps including rotations.
    """
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    
    if dir == 4:
        return [(loc[0], loc[1], 1, currDir)]
    
    if dir == currDir:
        new_loc = (loc[0] + directions[dir][0], loc[1] + directions[dir][1])
        return [(new_loc[0], new_loc[1], 1, currDir)]
    else:
        rotation_steps = []
        while currDir != dir:
            currDir = (currDir + 1) % 4
            rotation_steps.append((loc[0], loc[1], 1, currDir))
        new_loc = (loc[0] + directions[dir][0], loc[1] + directions[dir][1])
        rotation_steps.append((new_loc[0], new_loc[1], 1, currDir))
        return rotation_steps


def is_goal_constrained(goal_loc, time_step, constraint_table):
    """
    Check if there's a constraint on the goal location in the future.
    
    Parameters:
    goal_loc (tuple): Goal location (x, y).
    time_step (int): Current time step.
    constraint_table (dict): Constraint table.
    
    Returns:
    bool: True if there is a constraint on the goal location, False otherwise.
    """
    constraints = [c for t, c in constraint_table.items() if t > time_step]
    constraints = flatten_constraints(constraints)
    for c in constraints:
        if [goal_loc] == c['loc'] and c['final']:
            return True
    return False


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    """
    Check if the move to the next location is constrained.
    
    Parameters:
    curr_loc (tuple): Current location (x, y).
    next_loc (tuple): Next location (x, y).
    next_time (int): Next time step.
    constraint_table (dict): Constraint table.
    
    Returns:
    bool: True if the move is constrained, False otherwise.
    """
    if next_time in constraint_table:
        constraints = constraint_table[next_time]
        for c in constraints:
            if [next_loc] == c['loc'] or [curr_loc, next_loc] == c['loc']:
                return True
    else:
        constraints = [c for t, c in constraint_table.items() if t < next_time]
        constraints = flatten_constraints(constraints)
        for c in constraints:
            if [next_loc] == c['loc'] and c['final']:
                return True
    return False


def build_constraint_table(constraints, agent):
    """
    Build a constraint table for the given agent.
    
    Parameters:
    constraints (list): List of constraints.
    agent (int): Agent index.
    
    Returns:
    dict: Constraint table.
    """
    c_table = dict()
    for c in constraints:
        if 'positive' not in c:
            c['positive'] = False
        if c['agent'] == agent:
            timestep = c['timestep']
            if timestep not in c_table:
                c_table[timestep] = [c]
            else:
                c_table[timestep].append(c)
    return c_table


def pop_node(open_list):
    """
    Pop the node with the highest priority from the open list.
    
    Parameters:
    open_list (list): Open list.
    
    Returns:
    dict: Node with the highest priority.
    """
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """
    Compare two nodes based on their cost.
    
    Parameters:
    n1 (dict): First node.
    n2 (dict): Second node.
    
    Returns:
    bool: True if n1 is better than n2, False otherwise.
    """
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def compute_heuristics(grid_map, goal):
    """
    Compute heuristics using Dijkstra's algorithm.
    
    Parameters:
    grid_map (list): Grid map.
    goal (tuple): Goal location (x, y).
    
    Returns:
    dict: Heuristic values for each location.
    """
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0, 'path': [goal]}
    heapq.heappush(open_list, (root['cost'], goal))
    closed_list[goal] = root

    while open_list:
        cost, loc = heapq.heappop(open_list)
        for direction in range(4):
            neighbour_loc = move(loc, direction)
            neighbour_cost = cost + 1
            if neighbour_loc[0] < 0 or neighbour_loc[0] >= len(grid_map) or neighbour_loc[1] < 0 or neighbour_loc[1] >= len(grid_map[0]):
                continue
            if grid_map[neighbour_loc[0]][neighbour_loc[1]]:
                continue
            neighbour = {'loc': neighbour_loc, 'cost': neighbour_cost, 'path': closed_list[loc]['path'] + [neighbour_loc]}
            if neighbour_loc in closed_list:
                existing_node = closed_list[neighbour_loc]
                if existing_node['cost'] > neighbour_cost:
                    closed_list[neighbour_loc] = neighbour
                    heapq.heappush(open_list, (neighbour_cost, neighbour_loc))
            else:
                closed_list[neighbour_loc] = neighbour
                heapq.heappush(open_list, (neighbour_cost, neighbour_loc))

    h_values = {loc: {'cost': node['cost'], 'path': node['path']} for loc, node in closed_list.items()}
    return h_values


def a_star(grid_map, start_loc, goal_loc, h_values, agent, constraints):
    """
    A* search algorithm with space-time constraints.
    
    Parameters:
    grid_map (list): Grid map.
    start_loc (tuple): Start location (x, y).
    goal_loc (tuple): Goal location (x, y).
    h_values (dict): Heuristic values.
    agent (int): Agent index.
    constraints (list): List of constraints.
    
    Returns:
    list: Path from start to goal.
    """
    open_list = []
    closed_list = dict()
    h_value = h_values[start_loc]['cost']
    c_table = build_constraint_table(constraints, agent)

    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'direction': 0, 'time': 0}
    heapq.heappush(open_list, (root['g_val'] + root['h_val'], root['h_val'], root['loc'], root))
    closed_list[(start_loc, 0)] = root
    max_map_width = max(len(row) for row in grid_map)

    while open_list:
        curr = pop_node(open_list)
        if curr['loc'] == goal_loc and not is_goal_constrained(goal_loc, curr['time'], c_table):
            return get_path(curr)
        
        for direction in range(5):
            neighbour_steps = moveRotation(curr['loc'], direction, curr['direction'])
            for step in neighbour_steps:
                neighbour_loc = step[:2]
                if neighbour_loc[0] < 0 or neighbour_loc[1] < 0 or neighbour_loc[0] >= len(grid_map) or neighbour_loc[1] >= max_map_width or grid_map[neighbour_loc[0]][neighbour_loc[1]]:
                    continue
                neighbour = {'loc': neighbour_loc, 'g_val': curr['g_val'] + 1, 'h_val': h_values[neighbour_loc]['cost'], 'parent': curr, 'direction': step[3], 'time': curr['time'] + step[2]}
                if is_constrained(curr['loc'], neighbour_loc, neighbour['time'], c_table):
                    continue
                if (neighbour_loc, neighbour['time']) in closed_list:
                    existing_node = closed_list[(neighbour_loc, neighbour['time'])]
                    if compare_nodes(neighbour, existing_node):
                        closed_list[(neighbour_loc, neighbour['time'])] = neighbour
                        heapq.heappush(open_list, (neighbour['g_val'] + neighbour['h_val'], neighbour['h_val'], neighbour_loc, neighbour))
                else:
                    closed_list[(neighbour_loc, neighbour['time'])] = neighbour
                    heapq.heappush(open_list, (neighbour['g_val'] + neighbour['h_val'], neighbour['h_val'], neighbour_loc, neighbour))
    
    return None