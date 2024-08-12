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
        # problem.heuristicInfo[(position,food)] = len(astar(prob))
        # problem.heuristicInfo[(position, food)] = len(dijkstra(prob))
        problem.heuristicInfo[(position, food)] = len(bfs(prob))
        # problem.heuristicInfo[(position, food)] = len(dfs(prob))

        # return len(astar(prob))  # 442
        # return len(dijkstra(prob))  #442
        return len(bfs(prob)) # 442
        # return len(dfs(prob))   # 380

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
    pacman_positions, food_grid = problem.getStartState()

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
cbs = conflictBasedSearch
dijkstra = dijkstraSearch
