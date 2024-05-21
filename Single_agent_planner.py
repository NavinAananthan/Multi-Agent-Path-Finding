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
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path




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




def is_constrained(curr_loc,next_loc,next_time,constraint_table):
    '''
    Check if there is any constraints on the neighbouring node if it is a vertex or edge constraint,
    It also checks if it has reached the Final destination on the next_location
    '''
    if next_time in constraint_table:
        constraints = constraint_table[next_time]
        for c in constraints:
            if [next_loc] == c['loc'] or [curr_loc,next_loc] == c['loc']:
                return True
    else:
        constraints = [c for t, c in constraint_table.items() if t < next_time]
        constraints = flatten_constraints(constraints)
        for c in constraints:
            if [next_loc] == c['loc'] and c['final']:
                return True
    return False



def build_constarint_table(constraints, agent):
    '''
    Return a table that constains the list of constraints of the given agent for each time step.
    '''
    c_table = dict()

    for c in constraints:
        if not 'positive' in c.keys():
            c['positive'] = False
        if c['agent'] == agent:
            timestep = c['timestep']
            if timestep not in c_table:
                c_table[timestep] = [c]
            else:
                c_table[timestep].append(c)


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
    c_table = build_constarint_table(constraints,agent)

    root = {'loc':start_loc,
            'g_val':0,
            'h_val':h_value,
            'parent':None,
            'time':0}

    heapq.heappush(open_list,(root['g_val'] + root['h_val'], root['h_val'], root['loc'], root))
    closed_list[(start_loc,0)] = root
    #max_map_width = max([len(e) for e in grid_map])

    while len(open_list)>0:
        curr = pop_node(open_list)
        
        # Checking the Goal constraints
        if curr['loc'] == goal_loc and not is_goal_constrained(goal_loc,curr['time'],c_table):
            return get_path(curr)
        
        for direction in range(5):
            # This is based on exploring the neighbouring nodes
            if direction < 4:
                neighbour_loc = move(curr['loc'],direction)
                # Check if the neighbour node is inside the grid_map and it is not a obstacle
                if neighbour_loc[0] < 0 or neighbour_loc[0] >= len(grid_map) or neighbour_loc[1] < 0 or neighbour_loc[1] >= len(grid_map[0]) or grid_map[neighbour_loc[0]][neighbour_loc[1]]:
                    continue

                neighbour = {
                    'loc':neighbour_loc,
                    'g_val':curr['g_val']+1,
                    'h_val':h_values[neighbour_loc],
                    'parent':curr,
                    'time':curr['time']+1
                }
            # This is when the currnode doesnt move and is still
            else:
                neighbour = {
                    'loc':curr['loc'],
                    'g_val':curr['g_val']+1,
                    'h_val':curr['h_val'],
                    'parent':curr,
                    'time':curr['time']+1
                }

            # we need to check if the neighbouring node violates any constarint
            if is_constrained(curr['loc'],neighbour['loc'],neighbour['time'],c_table):
                continue
            if (neighbour['loc'],neighbour['time']) in closed_list:
                    existing_node = closed_list[(neighbour['loc'],neighbour['time'])]
                    # Comparing the heuristic value for the existing node and current node
                    if compare_nodes(neighbour,existing_node):
                        closed_list[(neighbour['loc'], neighbour['time'])] = neighbour
                        heapq.heappush(open_list,(neighbour['g_val'] + neighbour['h_val'], neighbour['h_val'], neighbour['loc'], neighbour))
                    else:
                        closed_list[(neighbour['loc'], neighbour['time'])] = neighbour
                        heapq.heappush(open_list,(neighbour['g_val'] + neighbour['h_val'], neighbour['h_val'], neighbour['loc'], neighbour))

    return None


                



