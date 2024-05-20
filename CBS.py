import heapq
import random
import time as timer
from collections import defaultdict
import logging

logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
console_handler = logging.StreamHandler()
logger.addHandler(console_handler)

DEBUG = True


# Basic movement function for grid-based navigation
def move(loc, direction):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    return loc[0] + directions[direction][0], loc[1] + directions[direction][1]


# Calculate total path cost
def getCost(paths):
    return sum(len(p) - 1 for p in paths)


# Get the location at a specific time in the path
def getLocation(path, time):
    return path[time] if 0 <= time < len(path) else path[-1]


# Extract the path from the goal node
def getPath(goal):
    path = []
    while goal:
        path.append(goal['loc'])
        goal = goal['parent']
    return path[::-1]


# Normalize the lengths of two paths by extending them with the last position
def normalizePaths(pathA, pathB):
    path1, path2 = pathA.copy(), pathB.copy()
    while len(path1) < len(path2):
        path1.append(path1[-1])
    while len(path2) < len(path1):
        path2.append(path2[-1])
    return path1, path2


# Detect collisions between two paths
def detectCollisions(pathA, pathB):
    path1, path2 = normalizePaths(pathA, pathB)
    for t in range(len(path1)):
        loc1, loc2 = getLocation(path1, t), getLocation(path2, t)
        if loc1 == loc2:
            return {'loc': [loc1], 'timestep': t, 'type': 'vertex'}
        if t < len(path1) - 1 and loc1 == getLocation(path2, t + 1) and loc2 == getLocation(path1, t + 1):
            return {'loc': [loc1, loc2], 'timestep': t + 1, 'type': 'edge'}
    return None


# Detect all collisions in a set of paths
def detectAllCollisions(paths):
    collisions = []
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            coll = detectCollisions(paths[i], paths[j])
            if coll:
                coll['a1'] = i
                coll['a2'] = j
                collisions.append(coll)
    return collisions


# Create constraints from a collision
def splittingNode(collision):
    constraints = []
    if collision['type'] == 'vertex':
        constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': True})
        constraints.append({'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': True})
    elif collision['type'] == 'edge':
        constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']})
        constraints.append({'agent': collision['a2'], 'loc': list(reversed(collision['loc'])), 'timestep': collision['timestep']})
    return constraints


# Create disjoint constraints from a collision
def disjointSplitting(collision):
    choice = random.randint(0, 1)
    agent = [collision['a1'], collision['a2']][choice]
    loc = collision['loc'] if choice == 0 else list(reversed(collision['loc']))
    return [
        {'agent': agent, 'loc': loc, 'timestep': collision['timestep'], 'positive': True},
        {'agent': agent, 'loc': loc, 'timestep': collision['timestep'], 'positive': False}
    ]


# Build a constraint table for an agent
def buildConstraintTable(constraints, agent):
    c_table = defaultdict(list)
    for c in constraints:
        if c['agent'] == agent:
            c_table[c['timestep']].append(c)

    if(DEBUG):
        print(c_table)
    return c_table


# Check if a move is constrained
def isConstrained(currLoc, nextLoc, nextTime, constraintTable):
    if nextTime in constraintTable:
        for c in constraintTable[nextTime]:
            if [nextLoc] == c['loc'] or [currLoc, nextLoc] == c['loc']:
                return True
    return False



# Check if the goal is constrained at a specific timestep
def isGoalConstrained(goalLoc, timestep, constraintTable):
    for t in range(timestep + 1, max(constraintTable.keys(), default=0) + 1):
        if any(c['loc'] == [goalLoc] for c in constraintTable[t]):
            return True
    return False


# Priority queue operations for A* search
def pushNode(open_list, node):
    heapq.heappush(open_list, (node['gVal'] + node['hVal'], node['hVal'], node['loc'], node))


def popNode(open_list):
    return heapq.heappop(open_list)[3]


# A* search algorithm
def aStar(map, startLoc, goalLoc, agent, constraints):
    openList = []
    closedList = {}
    root = {'loc': startLoc, 'gVal': 0, 'hVal': abs(startLoc[0] - goalLoc[0]) + abs(startLoc[1] - goalLoc[1]), 'parent': None, 'time': 0}
    pushNode(openList, root)
    closedList[(startLoc, 0)] = root
    logger.debug("***************************** Closed List *****************************")
    logger.info(closedList)
    
    
    while openList:
        curr = popNode(openList)
        if curr['loc'] == goalLoc and not isGoalConstrained(goalLoc, curr['time'], buildConstraintTable(constraints, agent)):
            return getPath(curr)
        for direction in range(5):
            if direction < 4:
                childLoc = move(curr['loc'], direction)
                if not (0 <= childLoc[0] < len(map) and 0 <= childLoc[1] < len(map[0]) and not map[childLoc[0]][childLoc[1]]):
                    continue
                child = {'loc': childLoc, 'gVal': curr['gVal'] + 1, 'hVal': abs(childLoc[0] - goalLoc[0]) + abs(childLoc[1] - goalLoc[1]), 'parent': curr, 'time': curr['time'] + 1}
            else:
                child = {'loc': curr['loc'], 'gVal': curr['gVal'] + 1, 'hVal': curr['hVal'], 'parent': curr, 'time': curr['time'] + 1}
            if isConstrained(curr['loc'], child['loc'], child['time'], buildConstraintTable(constraints, agent)):
                continue
            if (child['loc'], child['time']) in closedList:
                existing = closedList[(child['loc'], child['time'])]
                if child['gVal'] + child['hVal'] < existing['gVal'] + existing['hVal']:
                    pushNode(openList, child)
                    closedList[(child['loc'], child['time'])] = child
            else:
                pushNode(openList, child)
                closedList[(child['loc'], child['time'])] = child
    return None


# CBS solver class
class CBSSolver:
    def __init__(self, map, starts, goals, maxTime=None):
        self.startTime = timer.time()
        self.map = map
        self.starts = starts
        self.goals = goals
        self.numOfAgents = len(goals)
        self.num_generated = 0
        self.num_expanded = 0
        self.CPU_time = 0
        self.maxTime = maxTime if maxTime else float('inf')
        self.openList = []

    def pushNode(self, node):
        heapq.heappush(self.openList, (node['cost'], self.num_generated, node))
        self.num_generated += 1

    def popNode(self):
        _, _, node = heapq.heappop(self.openList)
        self.num_expanded += 1
        return node

    def findSolution(self, disjoint=True):
        constraints = []
        # Finding the paths for each agent
        startPaths = [aStar(self.map, self.starts[i], self.goals[i], i, constraints) for i in range(self.numOfAgents)]
        logger.debug("\n---------------------------------- Paths For multiple Agents ----------------------------------")
        logger.info(startPaths)
        logger.warning('\n')

        root = {'cost': getCost(startPaths), 'constraints': constraints, 'paths': startPaths, 'collisions': detectAllCollisions(startPaths)}
        self.pushNode(root)
        
        while self.openList and (timer.time() - self.startTime < self.maxTime):
            P = self.popNode()
            if not P['collisions']:
                self.CPU_time = timer.time() - self.startTime
                return P['paths']
            collision = P['collisions'][0]
            newConstraints = disjointSplitting(collision) if disjoint else splittingNode(collision)
            
            for constraint in newConstraints:
                Q = {'constraints': P['constraints'] + [constraint], 'paths': P['paths'][:]}
                agent = constraint['agent']
                path = aStar(self.map, self.starts[agent], self.goals[agent], agent, Q['constraints'])
                if path:
                    Q['paths'][agent] = path
                    Q['collisions'] = detectAllCollisions(Q['paths'])
                    Q['cost'] = getCost(Q['paths'])
                    self.pushNode(Q)
        self.CPU_time = timer.time() - self.startTime
        return None
    


if __name__ == "__main__":
    

    map = [
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0]
    ]
    
    # Example starts and goals
    starts = [(2, 0), (2, 3)]
    goals = [(2, 4), (2, 3)]
    
    # Create CBS solver instance
    solver = CBSSolver(map, starts, goals)
    
    # Find solution
    solution = solver.findSolution(disjoint=True)
    
    # if solution:
    #     print("Solution found:")
    #     for agent, path in enumerate(solution):
    #         print(f"Agent {agent}: {path}")
    # else:
    #     print("No solution found within the time limit.")
    