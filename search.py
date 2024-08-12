# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
import itertools
from game import Grid
from searchProblems import nullHeuristic,PositionSearchProblem,ConstrainedAstarProblem

### You might need to use
from copy import deepcopy
import collections
from searchProblems import FoodSearchProblem
def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    queue = collections.deque()
    startState = problem.getStartState()
    queue.append((startState, []))
    visited = set()

    while queue:
        state, path = queue.popleft()
        if state in visited:
            continue
        visited.add(state)
        if problem.isGoalState(state):
            return path
        for succState, succAction, succCost in problem.getSuccessors(state):
            if succState not in visited:
                queue.append((succState, path + [succAction]))
    return None # Goal not found
    # util.raiseNotDefined()


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    myPQ = util.PriorityQueue()
    startState = problem.getStartState()
    startNode = (startState, 0, [])
    myPQ.push(startNode, heuristic(startState, problem))
    best_g = dict()
    while not myPQ.isEmpty():
        node = myPQ.pop()
        state, cost, path = node
        if (not state in best_g) or (cost < best_g[state]):
            best_g[state] = cost
            if problem.isGoalState(state):
                return path
            for succ in problem.getSuccessors(state):
                succState, succAction, succCost = succ
                new_cost = cost + succCost
                newNode = (succState, new_cost, path + [succAction])
                myPQ.push(newNode, heuristic(succState, problem) + new_cost)

    return None  # Goal not found


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    stack = [(problem.getStartState(), [], 0)]  # Stack holds (state, path, cost)
    visited = set()
    while stack:
        state, path, cost = stack.pop()
        if state in visited:
            continue
        visited.add(state)
        if problem.isGoalState(state):
            return path
        for succState, succAction, succCost in problem.getSuccessors(state):
            if succState not in visited:
                stack.append((succState, path + [succAction], cost + succCost))

    return None # Goal not found

    # util.raiseNotDefined()


def dijkstraSearch(problem):
    """Search the node with the lowest cost first (Dijkstra's Algorithm)."""
    myPQ = util.PriorityQueue()
    startState = problem.getStartState()
    startNode = (startState, 0, [])
    myPQ.push(startNode, 0)  # No heuristic, only actual cost
    best_g = dict()

    while not myPQ.isEmpty():
        node = myPQ.pop()
        state, cost, path = node
        if (not state in best_g) or (cost < best_g[state]):
            best_g[state] = cost
            if problem.isGoalState(state):
                return path
            for succ in problem.getSuccessors(state):
                succState, succAction, succCost = succ
                new_cost = cost + succCost
                newNode = (succState, new_cost, path + [succAction])
                myPQ.push(newNode, new_cost)  # No heuristic component

    return None

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    util.raiseNotDefined()


class MyFoodSearchProblem(FoodSearchProblem):
    def __init__(self, position, food, walls):
        self.start = (position, food)
        self.heuristicInfo = {}
        self.walls = walls
        self._expanded = 0

    def getStartState(self):
        return self.start

def foodHeuristic(state, problem):
    """
    Your heuristic for the FoodSearchProblem goes here.

    This heuristic must be consistent to ensure correctness.  First, try to come
    up with an admissible heuristic; almost all admissible heuristics will be
    consistent as well.

    If using A* ever finds a solution that is worse uniform cost search finds,
    your heuristic is *not* consistent, and probably not admissible!  On the
    other hand, inadmissible or inconsistent heuristics may find optimal
    solutions, so be careful.

    The state is a tuple ( pacmanPosition, foodGrid ) where foodGrid is a Grid
    (see game.py) of either True or False. You can call foodGrid.asList() to get
    a list of food coordinates instead.

    If you want access to info like walls, capsules, etc., you can query the
    problem.  For example, problem.walls gives you a Grid of where the walls
    are.

    If you want to *store* information to be reused in other calls to the
    heuristic, there is a dictionary called problem.heuristicInfo that you can
    use. For example, if you only want to count the walls once and store that
    value, try: problem.heuristicInfo['wallCount'] = problem.walls.count()
    Subsequent calls to this heuristic can access
    problem.heuristicInfo['wallCount']
    """
    "*** YOUR CODE HERE for TASK1 ***"
    pacmanPosition, foodGrid = state
    foodList = foodGrid.asList()
    maxDistance = 0

# This function cost 719
#     minDistance =float('inf')
#     if not foodList:
#         return 0
#     for food in foodList:
#         distanceToPacman = getDistanceBetweenTwoPos(pacmanPosition, food, problem)
#         if distanceToPacman < minDistance:
#             minDistance = distanceToPacman
#         for otherFood in foodList:
#             if food != otherFood:
#                 currentDistance = getDistanceBetweenTwoPos(food, otherFood, problem)
#                 if currentDistance > maxDistance:
#                     maxDistance = currentDistance
#     return  minDistance + maxDistance

    # This function cost 376
    if foodList == []:
        return 0
    else:
        for i in range(len(foodList)):
            for j in range(len(foodList)):
                # calculate every pair's distance with getDistanceBetweenTwoPos
                currentDistance = getDistanceBetweenTwoPos(foodList[i], foodList[j], problem)
                if currentDistance > maxDistance:
                    # find the furrest food and calculate its distance between original position
                    maxDistance = currentDistance
                    furthest = (foodList[i], foodList[j])

        distance_to_furthest = []
        for food in furthest:
            distance_to_furthest.append(getDistanceBetweenTwoPos(pacmanPosition, food, problem))
        min_distance_to_furthest = min(distance_to_furthest)
        res = maxDistance + min_distance_to_furthest
    return res

def getDistanceBetweenTwoPos(position, food, problem):
    # Here I tried different search algorithm like bfs, dfs, dijkstra and Astar
    try:
        return problem.heuristicInfo[(position, food)]
    except:
        foodGrid = Grid(problem.walls.width, problem.walls.height, False)
        foodGrid[food[0]][food[1]] = True
        prob = MyFoodSearchProblem(position, foodGrid, problem.walls)
        problem.heuristicInfo[(position,food)] = len(astar(prob))
        # problem.heuristicInfo[(position, food)] = len(dijkstra(prob))
        # problem.heuristicInfo[(position, food)] = len(bfs(prob))
        # problem.heuristicInfo[(position, food)] = len(dfs(prob))

        return len(astar(prob))  # 442   376
        # return len(dijkstra(prob))  #442   376
        # return len(bfs(prob)) # 442    376
        # return len(dfs(prob))   # 380    1419


from searchProblems import MAPFProblem


def conflictBasedSearch(problem: MAPFProblem):
    """
        Conflict-based search algorithm.
        Input: MAPFProblem
        Output(IMPORTANT!!!): A dictionary stores path for each pacman as a list {pacman_name: [a1, a2, ...]}.

        A search state in this problem is a tuple ( pacmanPosition, foodGrid ) where
          pacmanPosition: a tuple (x,y) of integers specifying Pacman's position
          foodGrid:       a Grid (see game.py) of either True or False, specifying remaining food

        Hints:
            You should model the constrained Astar problem as a food search problem instead of a position search problem,
            you can use: ConstrainedAstarProblem
            The stop action may also need to be considered as a valid action for finding an optimal solution
            Also you should model the constraints as vertex constraints and edge constraints

    """
    "*** YOUR CODE HERE for TASK2 ***"
    def initialize_root():
        pacman_positions, food_grid = problem.getStartState()
        agentList = list(pacman_positions.keys())
        root = {
            'constraints': {},
            'path': {},
            'solution': {},
            'cost': 0
        }
        for agent in agentList:
            astar_cbs = AStarSearchCBS(problem, agent, root['constraints'])
            path, solution = astar_cbs.run_search()
            root['path'][agent] = path
            root['solution'][agent] = solution
        root['cost'] = sum(len(path) for path in root['solution'].values())
        return root

    def process_conflict(node, conflict):
        conflictPacmans = conflict['pacmans']
        conflictPosition = conflict['position']
        conflictTime = conflict['time']
        new_nodes = []
        for pacman in conflictPacmans:
            new_constraints = node['constraints'].copy()
            if not conflict['swap']:
                new_constraints[(pacman, conflictPosition, conflictTime, False)] = conflictTime
            else:
                new_constraints[(pacman, conflictPosition[pacman], conflictTime, True)] = conflictTime

            new_node = {
                'constraints': new_constraints,
                'path': node['path'].copy(),
                'solution': node['solution'].copy(),
                'cost': 0
            }
            max_length = max(len(sol) for sol in new_node['solution'].values())
            astar_cbs1 = AStarSearchCBS(problem, pacman, new_constraints, goalTime=max_length)
            new_solution = astar_cbs1.run_search()
            if new_solution:
                path, solution = new_solution
                new_node['path'][pacman] = path
                new_node['solution'][pacman] = solution
                new_node['cost'] = sum(len(sol) for sol in new_node['solution'].values())
                new_nodes.append(new_node)
        return new_nodes

    root = initialize_root()
    OPEN = util.PriorityQueue()
    OPEN.push(root, root['cost'])

    while not OPEN.isEmpty():
        node = OPEN.pop()
        detector = ConflictDetector(node['path'])
        conflict = detector.detect_conflict()

        if not conflict:
            print("final solution:", node['solution'])
            print("final path:", node['path'])
            return node['solution']

        new_nodes = process_conflict(node, conflict)
        for new_node in new_nodes:
            OPEN.push(new_node, new_node['cost'])

    return None

class AStarSearchCBS:
    def __init__(self, problem, agent, constraints, goalTime=-1, heuristic=foodHeuristic):
        self.problem = problem
        self.agent = agent
        self.constraints = constraints
        self.goalTime = goalTime
        self.heuristic = heuristic
        self.visited = set()
        self.myPQ = util.PriorityQueue()
        self.startState = self.problem.getStartState()
        self.goalState = self.find_goal_state()

    def find_goal_state(self):
        foodGrid = self.startState[1]
        for x in range(foodGrid.width):
            for y in range(foodGrid.height):
                if foodGrid[x][y] == self.agent:
                    return (x, y)
        return None

    def run_search(self):
        startPosition = self.startState[0][self.agent]
        startNode = (self.startState, 0, [startPosition], [], 0)
        self.myPQ.push(startNode, 0)

        while not self.myPQ.isEmpty():
            currentNode = self.myPQ.pop()
            if self.process_current_node(currentNode):
                return currentNode[2], currentNode[3]
        return None, None

    def process_current_node(self, currentNode):
        currentState, currentCost, currentPath, currentSolution, currentTime = currentNode
        currentPosition = currentState[0][self.agent]

        if (currentPosition, currentTime) in self.visited:
            return False

        self.visited.add((currentPosition, currentTime))

        if self.goalState in currentPath:
            if self.goalTime == -1 or self.can_early_stop(currentPosition, currentTime):
                return True

        self.expand_successors(currentNode)
        return False

    def can_early_stop(self, currentPosition, currentTime):
        for key, _ in self.constraints.items():
            _, constraintsPos, constraintsTime, isSwap = key
            if constraintsTime > currentTime:
                if (isSwap and currentPosition in constraintsPos) or (not isSwap and currentPosition == constraintsPos):
                    return False
        return True

    def expand_successors(self, currentNode):
        currentState, currentCost, currentPath, currentSolution, currentTime = currentNode

        for nextState, action, stepCost in self.problem.getSuccessors(currentState):
            nextPosition = nextState[0][self.agent]
            if self.is_constrained(currentPosition=currentState[0][self.agent], nextPosition=nextPosition, currentTime=currentTime):
                continue

            newCost = currentCost + stepCost
            newPath = currentPath + [nextPosition]
            newSolution = currentSolution + [action[self.agent]]
            newTime = currentTime + 1
            heuristicCost = util.manhattanDistance(nextPosition, self.goalState)
            totalCost = newCost + heuristicCost
            newNode = (nextState, newCost, newPath, newSolution, newTime)
            self.myPQ.push(newNode, totalCost)

    def is_constrained(self, currentPosition, nextPosition, currentTime):
        return ((self.agent, nextPosition, currentTime + 1, False) in self.constraints.keys() or
                (self.agent, (currentPosition, nextPosition), currentTime + 1, True) in self.constraints.keys())



class ConflictDetector:
    def __init__(self, paths):
        self.paths = paths
        self.max_length = max(len(path) for path in paths.values())

    def detect_conflict(self):
        for t in range(self.max_length):
            all_positions = self.get_all_positions_at_time(t)
            conflict = self.check_vertex_conflict(all_positions, t)
            if conflict:
                return conflict
            conflict = self.check_swap_conflict(all_positions, t)
            if conflict:
                return conflict
        return {}

    def get_all_positions_at_time(self, t):
        return {
            agent: (path[t] if t < len(path) else path[-1])
            for agent, path in self.paths.items()
        }

    # check vertext conflict
    def check_vertex_conflict(self, positions, time):
        seen_positions = {}
        for agent, pos in positions.items():
            if pos in seen_positions:
                return {
                    'pacmans': [seen_positions[pos], agent],
                    'position': pos,
                    'time': time,
                    'swap': False
                }
            seen_positions[pos] = agent
        return None

    # check swap conflict
    def check_swap_conflict(self, positions, time):
        for pacA, posA in positions.items():
            for pacB, posB in positions.items():
                if pacA != pacB:
                    past_posA = self.get_past_position(pacA, time)
                    past_posB = self.get_past_position(pacB, time)
                    if posA == past_posB and posB == past_posA:
                        return {
                            'pacmans': [pacA, pacB],
                            'position': {pacA: (past_posA, posA), pacB: (past_posB, posB)},
                            'time': time,
                            'swap': True
                        }
        return None

    def get_past_position(self, agent, time):
        if time > 0:
            return self.paths[agent][time - 1] if time <= len(self.paths[agent]) else self.paths[agent][-1]
        return self.paths[agent][0]


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
cbs = conflictBasedSearch
dijkstra = dijkstraSearch
