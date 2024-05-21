import heapq



def move(loc,direction):
    '''
    Adding the current location x and y to all fpur directions of x and y to get the neighbour nodes
    '''
    directions = [(0,-1),(1,0),(0,1),(-1,0)]
    return loc[0]+directions[direction][0] , loc[1]+directions[direction][1]


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


def compute_heuristics(map,goal):
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

            # Checking whether the neighbour's node is inside the map
            if neighbour_loc[0] < 0 or neighbour_loc[0] >= len(map) or neighbour_loc[1] < 0 or neighbour_loc[1] >= len(map[0]):
                continue
            # Checking if the neighbour node is not an obstacle
            if map[neighbour_loc[0]][neighbour_loc[1]]:
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


def a_star(map, start_loc, goal_loc, h_values, agent, constraints):
    '''
    This is the implementation of Astar algorithm using constraints we extend it in Space-Time domain
    '''
    
    open_list = []
    closed_list = dict()
    h_value = h_values[start_loc]['cost']
    c_table = build_constarint_table(constraints,agent)

    root = {'loc':start_loc,'g_val':0,'h_val':h_value,'parent':None,'time':0}

    heapq.heappush(open_list,(root['g_val'] + root['h_val'], root['h_val'], root['loc'], root))
    closed_list[(start_loc,0)] = root
    max_map_width = max([len(e) for e in map])

    while len(open_list)>0:
        curr = pop_node(open_list)
        


