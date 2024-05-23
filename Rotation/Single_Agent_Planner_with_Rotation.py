import heapq



def move(loc,direction):
    '''
    Adding the current location x and y to all fpur directions of x and y to get the neighbour nodes
    '''
    directions = [(0,-1),(1,0),(0,1),(-1,0)]
    return loc[0]+directions[direction][0] , loc[1]+directions[direction][1]


def flatten_constraints(constraints):
    '''
    This is to flatten the constraints when pass it as a list of list
    '''
    constraints = []
    for constr_list in constraints:
        for c in constr_list:
            constraints.append(c)
    return constraints


def get_path(goal_node):
    '''
    This is to give the path from start to goal node by reversing it
    '''
    path = []
    curr = goal_node
    while curr is not None:
        path.append((curr['loc'],curr['time']))
        curr = curr['parent']
    path.reverse()
    return path



def moveRotation(loc, dir, currDir):
    # Directions: 0 = North, 1 = East, 2 = South, 3 = West
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    #print('In Move Rotation, current params: ', loc, dir, currDir)
    
    # If dir is 4, it means the agent should stay in the same place (waiting)
    if dir == 4:
        return [(loc[0], loc[1], 1, currDir)]
    
    if dir == currDir:
        # Moving in the same direction as the current facing direction
        new_loc = (loc[0] + directions[dir][0], loc[1] + directions[dir][1])
        return [(new_loc[0], new_loc[1], 1, currDir)]
    else:
        # Rotate to the new direction
        rotation_steps = []
        while currDir != dir:
            currDir = (currDir + 1) % 4
            rotation_steps.append((loc[0], loc[1], 1, currDir))
        new_loc = (loc[0] + directions[dir][0], loc[1] + directions[dir][1])
        rotation_steps.append((new_loc[0], new_loc[1], 1, currDir))
        return rotation_steps



def is_goal_constrained(goal_loc,time_step,constraint_table):
    '''
    check if there's a constraint on the goal in the future
    '''
    constraints = [c for t,c in constraint_table.items() if t>time_step]
    constraints  = flatten_constraints(constraints)
    for c in constraints:
        if [goal_loc] == c['loc'] and c['final']:
            return True
        return False




def is_constrained(curr_loc, next_loc, next_time, constraint_table):

    if next_time in constraint_table:
        constraints = constraint_table[next_time]
        for c in constraints:
            print(c)
            if [next_loc] == c['loc'] or [curr_loc, next_loc] == c['loc']:
                print(f'Constraint found: {curr_loc} -> {next_loc} at time {next_time}')
                return True
    else:
        constraints = [c for t, c in constraint_table.items() if t < next_time]
        constraints = flatten_constraints(constraints)
        for c in constraints:
            if [next_loc] == c['loc'] and c['final']:
                print(f'Final constraint found: {next_loc} at time {next_time}')
                return True
    return False



def build_constarint_table(constraints, agent):
    '''
    Return a table that constains the list of constraints of the given agent for each time step.
    '''
    c_table = dict()

    for c in constraints:
        if not 'positive' in c:
            c['positive'] = False
        if c['agent'] == agent:
            timestep = c['timestep']
            if timestep not in c_table:
                c_table[timestep] = [c]
            else:
                c_table[timestep].append(c)

    return c_table


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']



def compute_heuristics(grid_map,goal):
    '''
    We use Dijkstras algorithm to build a shortest path table to all reachable locations from Goal
    '''
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0, 'path': [goal]}

    heapq.heappush(open_list, (root['cost'], goal))
    closed_list[goal] = root

    while len(open_list) > 0:
        cost, loc = heapq.heappop(open_list)

        for direction in range(4):
            neighbour_loc = move(loc, direction)
            neighbour_cost = cost + 1

            # Checking whether the neighbour's node is inside the grid_map
            if neighbour_loc[0] < 0 or neighbour_loc[0] >= len(grid_map) or neighbour_loc[1] < 0 or neighbour_loc[1] >= len(grid_map[0]):
                continue
            # Checking if the neighbour node is not an obstacle
            if grid_map[neighbour_loc[0]][neighbour_loc[1]]:
                continue

            neighbour = {'loc': neighbour_loc, 'cost': neighbour_cost, 'path': closed_list[loc]['path'] + [neighbour_loc]}
            # Checking if the neighbour is already visited or not
            if neighbour_loc in closed_list:
                # Updating the path if there exists a cheaper path for the already existing node
                existing_node = closed_list[neighbour_loc]
                if existing_node['cost'] > neighbour_cost:
                    closed_list[neighbour_loc] = neighbour
                    heapq.heappush(open_list, (neighbour_cost, neighbour_loc))
            else:
                closed_list[neighbour_loc] = neighbour
                heapq.heappush(open_list, (neighbour_cost, neighbour_loc))


    # Building the heuristic table
    h_values = {loc: {'cost': node['cost'], 'path': node['path']} for loc, node in closed_list.items()}
    return h_values


def a_star(grid_map, start_loc, goal_loc, h_values, agent, constraints):
    '''
    This is the implementation of Astar algorithm using constraints we extend it in Space-Time domain
    '''
    
    open_list = []
    closed_list = dict()
    h_value = h_values[start_loc]['cost']
    #print(h_values)
    c_table = build_constarint_table(constraints,agent)


    root = {'loc':start_loc,
            'g_val':0,
            'h_val':h_value,
            'parent':None,
            'direction':0,
            'time':0}

    heapq.heappush(open_list,(root['g_val'] + root['h_val'], root['h_val'], root['loc'], root))
    closed_list[(start_loc,0)] = root
    max_map_width = max([len(e) for e in grid_map])

    while len(open_list)>0:
        curr = pop_node(open_list)
        
        # Checking the Goal constraints
        if curr['loc'] == goal_loc and not is_goal_constrained(goal_loc,curr['time'],c_table):
            return get_path(curr)
        
        for direction in range(5):
            neighbour_steps = moveRotation(curr['loc'],direction,curr['direction'])
            for step in neighbour_steps:
                neighbour_loc = step[:2]
            # Check if the neighbour node is inside the grid_map and it is not a obstacle
                if neighbour_loc[0] < 0 or neighbour_loc[1] < 0 or neighbour_loc[0] >= len(grid_map) or neighbour_loc[1] >= max_map_width or grid_map[neighbour_loc[0]][neighbour_loc[1]]:
                    continue

                neighbour = {
                    'loc':neighbour_loc,
                    'g_val':curr['g_val']+1,
                    'h_val':h_values[neighbour_loc]['cost'],
                    'parent':curr,
                    'direction':step[3],
                    'time':curr['time']+step[2]
                }

                # we need to check if the neighbouring node violates any constarint
                if is_constrained(curr['loc'],neighbour_loc,neighbour['time'],c_table):
                    continue
                if (neighbour_loc,neighbour['time']) in closed_list:
                        existing_node = closed_list[(neighbour_loc,neighbour['time'])]
                        # Comparing the heuristic value for the existing node and current node
                        if compare_nodes(neighbour,existing_node):
                            closed_list[(neighbour_loc, neighbour['time'])] = neighbour
                            heapq.heappush(open_list,(neighbour['g_val'] + neighbour['h_val'], neighbour['h_val'], neighbour_loc, neighbour))
                else:
                    closed_list[(neighbour_loc, neighbour['time'])] = neighbour
                    heapq.heappush(open_list,(neighbour['g_val'] + neighbour['h_val'], neighbour['h_val'], neighbour_loc, neighbour))

    return None


                
