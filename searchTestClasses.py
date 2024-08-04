# searchTestClasses.py
# --------------------
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


import datetime
import math
import re
import sys
import textwrap
import traceback

from func_timeout import func_timeout,FunctionTimedOut

# import project specific code
import layout
import pacman
import search
# import search_sol
import testClasses
# from search import SearchProblem
from searchProblems import SearchProblem
import searchProblems
import testSearch
# from search_sol import SearchProblem, validate

DEFAULT_TIMEOUT = 10


# helper function for printing solutions in solution files
def wrap_solution(solution):
    if type(solution) == type([]):
        return '\n'.join(textwrap.wrap(' '.join(solution)))
    else:
        return str(solution)

def followAction(state, action, problem):
  for successor1, action1, cost1 in problem.getSuccessors(state):
    if action == action1: return successor1
  return None

def followActionVerify(state_sol, state, action, problem_sol, problem):
    if state_sol != state:
        raise ValueError("The state is not expected after the transition")


    if len(problem.getSuccessors(state)) < len(problem_sol.getSuccessors(state)):
        raise ValueError("The number of successors are less than expected")
    elif len(problem.getSuccessors(state)) > len(problem_sol.getSuccessors(state)):
        raise ValueError("The number of successors are more than expected")


    new_state_sol, new_state = None, None
    for successor1, action1, cost1 in problem_sol.getSuccessors(state_sol):
        if action == action1: new_state_sol = successor1

    for successor1, action1, cost1 in problem.getSuccessors(state):
        if action == action1: new_state = successor1

    if new_state_sol and new_state:
        return new_state_sol, new_state
    else:
        raise ValueError("Your action might not applicable on the state")



def followPath(path, problem):
    state = problem.getStartState()
    states = [state]
    for action in path:
        state = followAction(state, action, problem)
        states.append(state)
    return states

def checkSolution(problem, path):
  state = problem.getStartState()
  trajs = [state]
  for action in path:
    state = followAction(state, action, problem)
    trajs.append(state)
  return problem.isGoalState(state), trajs

def checkSolutionVerify(problem_sol, path, problem):
    state_sol = problem_sol.getStartState()
    state = problem.getStartState()
    try:
        for action in path:
            state_sol, state = followActionVerify(state_sol, state, action, problem_sol, problem)
        return problem_sol.isGoalState(state_sol) and problem.isGoalState(state)
    except ValueError as e:
        raise ValueError(e.args)



def checkTrajs(sol, problem):
    pacman_positions, food_grid = problem.getStartState()
    walls = problem.walls
    trajs_dic = {}

    for p in pacman_positions:
        if not p in sol:
            return False


        x1, y1 = pacman_positions[p]
        x2, y2 = food_grid.asList(p)[0]
        new_food_grid = Grid(walls.width, walls.height)
        new_food_grid[x2][y2] = True
        prob = searchProblems.ConstrainedAstarProblem(pos=(x1, y1), food=new_food_grid, walls=walls)

        res, trajs_dic[p] = checkSolution(prob, sol[p])
        if not res:
            return False

    # No conflict
    if not validate(trajs_dic):
        return True

    return False

def validate(trajs):
    # vertex conflict
    t = 1
    flag = True

    while flag:
        poss = {}
        flag = False
        for p in trajs:
            if len(trajs[p]) > t:
                pos = trajs[p][t][0]
                flag = True
            else:
                pos = trajs[p][-1][0]

            if pos in poss:
                return p, poss[pos], pos, t
            poss[pos] = p

        t += 1

    # edge conflict
    t = 1
    flag = True

    while flag:
        edges = {}
        flag = False
        for p in trajs:
            if len(trajs[p]) > t:
                pre_pos = trajs[p][t - 1][0]
                pos = trajs[p][t][0]
                if (pos, pre_pos) in edges:
                    return p, edges[(pos, pre_pos)], pre_pos, pos, t - 1
                edges[(pre_pos, pos)] = p
                flag = True

        t += 1

    return None


# Search problem on a plain graph
class GraphSearch(SearchProblem):

    # Read in the state graph; define start/end states, edges and costs
    def __init__(self, graph_text):
        self.expanded_states = []
        lines = graph_text.split('\n')
        r = re.match('start_state:(.*)', lines[0])
        if r == None:
            print("Broken graph:")
            print('"""%s"""' % graph_text)
            raise Exception("GraphSearch graph specification start_state not found or incorrect on line 0")
        self.start_state = r.group(1).strip()
        r = re.match('goal_states:(.*)', lines[1])
        if r == None:
            print("Broken graph:")
            print('"""%s"""' % graph_text)
            raise Exception("GraphSearch graph specification goal_states not found or incorrect on line 1")
        goals = r.group(1).split()
        self.goals = [str.strip(g) for g in goals]
        self.successors = {}
        all_states = set()
        self.orderedSuccessorTuples = []
        for l in lines[2:]:
            if len(l.split()) == 3:
                start, action, next_state = l.split()
                cost = 1
            elif len(l.split()) == 4:
                start, action, next_state, cost = l.split()
            else:
                print("Broken graph:")
                print('"""%s"""' % graph_text)
                raise Exception("Invalid line in GraphSearch graph specification on line:" + l)
            cost = float(cost)
            self.orderedSuccessorTuples.append((start, action, next_state, cost))
            all_states.add(start)
            all_states.add(next_state)
            if start not in self.successors:
                self.successors[start] = []
            self.successors[start].append((next_state, action, cost))
        for s in all_states:
            if s not in self.successors:
                self.successors[s] = []

    # Get start state
    def getStartState(self):
        return self.start_state

    # Check if a state is a goal state
    def isGoalState(self, state):
        return state in self.goals

    # Get all successors of a state
    def getSuccessors(self, state):
        self.expanded_states.append(state)
        return list(self.successors[state])

    # Calculate total cost of a sequence of actions
    def getCostOfActions(self, actions):
        total_cost = 0
        state = self.start_state
        for a in actions:
            successors = self.successors[state]
            match = False
            for (next_state, action, cost) in successors:
                if a == action:
                    state = next_state
                    total_cost += cost
                    match = True
            if not match:
                print('invalid action sequence')
                sys.exit(1)
        return total_cost

    # Return a list of all states on which 'getSuccessors' was called
    def getExpandedStates(self):
        return self.expanded_states

    def __str__(self):
        print(self.successors)
        edges = ["%s %s %s %s" % t for t in self.orderedSuccessorTuples]
        return \
                """start_state: %s
                goal_states: %s
                %s""" % (self.start_state, " ".join(self.goals), "\n".join(edges))


def parseHeuristic(heuristicText):
    heuristic = {}
    for line in heuristicText.split('\n'):
        tokens = line.split()
        if len(tokens) != 2:
            print("Broken heuristic:")
            print('"""%s"""' % heuristicText)
            raise Exception("GraphSearch heuristic specification broken at tokens:" + str(tokens))
        state, h = tokens
        heuristic[state] = float(h)

    def graphHeuristic(state, problem=None):
        if state in heuristic:
            return heuristic[state]
        else:
            import pprint
            pp = pprint.PrettyPrinter(indent=4)
            print("Heuristic:")
            pp.pprint(heuristic)
            raise Exception("Graph heuristic called with invalid state: " + str(state))

    return graphHeuristic


class GraphSearchTest(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(GraphSearchTest, self).__init__(question, testDict)
        self.graph_text = testDict['graph']
        self.alg = testDict['algorithm']
        self.diagram = testDict['diagram']
        self.exactExpansionOrder = testDict.get('exactExpansionOrder', 'True').lower() == "true"
        if 'heuristic' in testDict:
            self.heuristic = parseHeuristic(testDict['heuristic'])
        else:
            self.heuristic = None

    # Note that the return type of this function is a tripple:
    # (solution, expanded states, error message)
    def getSolInfo(self, search):
        alg = getattr(search, self.alg)
        problem = GraphSearch(self.graph_text)
        if self.heuristic != None:
            solution = alg(problem, self.heuristic)
        else:
            solution = alg(problem)

        if type(solution) != type([]):
            return None, None, 'The result of %s must be a list. (Instead, it is %s)' % (self.alg, type(solution))

        return solution, problem.getExpandedStates(), None

    # Run student code.  If an error message is returned, print error and return false.
    # If a good solution is returned, printn the solution and return true; otherwise,
    # print both the correct and student's solution and return false.
    def execute(self, grades, moduleDict, solutionDict):
        search = moduleDict['search']
        searchAgents = moduleDict['searchAgents']
        gold_solution = [str.split(solutionDict['solution']), str.split(solutionDict['rev_solution'])]
        gold_expanded_states = [str.split(solutionDict['expanded_states']),
                                str.split(solutionDict['rev_expanded_states'])]

        solution, expanded_states, error = self.getSolInfo(search)
        if error != None:
            grades.addMessage('FAIL: %s' % self.path)
            grades.addMessage('\t%s' % error)
            return False

        if solution in gold_solution and (not self.exactExpansionOrder or expanded_states in gold_expanded_states):
            grades.addMessage('PASS: %s' % self.path)
            grades.addMessage('\tsolution:\t\t%s' % solution)
            grades.addMessage('\texpanded_states:\t%s' % expanded_states)
            return True
        else:
            grades.addMessage('FAIL: %s' % self.path)
            grades.addMessage('\tgraph:')
            for line in self.diagram.split('\n'):
                grades.addMessage('\t    %s' % (line,))
            grades.addMessage('\tstudent solution:\t\t%s' % solution)
            grades.addMessage('\tstudent expanded_states:\t%s' % expanded_states)
            grades.addMessage('')
            grades.addMessage('\tcorrect solution:\t\t%s' % gold_solution[0])
            grades.addMessage('\tcorrect expanded_states:\t%s' % gold_expanded_states[0])
            grades.addMessage('\tcorrect rev_solution:\t\t%s' % gold_solution[1])
            grades.addMessage('\tcorrect rev_expanded_states:\t%s' % gold_expanded_states[1])
            return False

    def writeSolution(self, moduleDict, filePath):
        search = moduleDict['search']
        searchAgents = moduleDict['searchAgents']
        # open file and write comments
        handle = open(filePath, 'w')
        handle.write('# This is the solution file for %s.\n' % self.path)
        handle.write('# This solution is designed to support both right-to-left\n')
        handle.write('# and left-to-right implementations.\n')

        # write forward solution
        solution, expanded_states, error = self.getSolInfo(search)
        if error != None: raise Exception("Error in solution code: %s" % error)
        handle.write('solution: "%s"\n' % ' '.join(solution))
        handle.write('expanded_states: "%s"\n' % ' '.join(expanded_states))

        # reverse and write backwards solution
        search.REVERSE_PUSH = not search.REVERSE_PUSH
        solution, expanded_states, error = self.getSolInfo(search)
        if error != None: raise Exception("Error in solution code: %s" % error)
        handle.write('rev_solution: "%s"\n' % ' '.join(solution))
        handle.write('rev_expanded_states: "%s"\n' % ' '.join(expanded_states))

        # clean up
        search.REVERSE_PUSH = not search.REVERSE_PUSH
        handle.close()
        return True


class PacmanSearchTest(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(PacmanSearchTest, self).__init__(question, testDict)
        self.layout_text = testDict['layout']
        self.alg = testDict['algorithm']
        self.layoutName = testDict['layoutName']

        # TODO: sensible to have defaults like this?
        self.leewayFactor = float(testDict.get('leewayFactor', '1'))
        self.costFn = eval(testDict.get('costFn', 'None'))
        self.searchProblemClassName = testDict.get('searchProblemClass', 'FoodSearchProblem')
        self.heuristicName = testDict.get('heuristic', None)

    def getSolInfo(self, search, searchAgents):
        alg = getattr(search, self.alg)
        lay = layout.Layout([l.strip() for l in self.layout_text.split('\n')])
        start_state = pacman.GameState()
        start_state.initialize(lay, 0)

        problemClass = getattr(searchAgents, self.searchProblemClassName)
        problemOptions = {}
        if self.costFn != None:
            problemOptions['costFn'] = self.costFn
        problem = problemClass(start_state, **problemOptions)
        heuristic = getattr(searchAgents, self.heuristicName) if self.heuristicName != None else None

        if heuristic != None:
            solution = alg(problem, heuristic)
        else:
            solution = alg(problem)

        if type(solution) != type([]):
            return None, None, 'The result of %s must be a list. (Instead, it is %s)' % (self.alg, type(solution))

        from game import Directions
        dirs = Directions.LEFT.keys()
        if [el in dirs for el in solution].count(False) != 0:
            return None, None, 'Output of %s must be a list of actions from game.Directions' % self.alg

        expanded = problem._expanded
        return solution, expanded, None

    def execute(self, grades, moduleDict, solutionDict):
        search = moduleDict['search']
        searchAgents = moduleDict['searchAgents']
        gold_solution = [str.split(solutionDict['solution']), str.split(solutionDict['rev_solution'])]
        gold_expanded = max(int(solutionDict['expanded_nodes']), int(solutionDict['rev_expanded_nodes']))

        solution, expanded, error = self.getSolInfo(search, searchAgents)
        if error != None:
            grades.addMessage('FAIL: %s' % self.path)
            grades.addMessage('%s' % error)
            return False

        # FIXME: do we want to standardize test output format?

        if solution not in gold_solution:
            grades.addMessage('FAIL: %s' % self.path)
            grades.addMessage('Solution not correct.')
            grades.addMessage('\tstudent solution length: %s' % len(solution))
            grades.addMessage('\tstudent solution:\n%s' % wrap_solution(solution))
            grades.addMessage('')
            grades.addMessage('\tcorrect solution length: %s' % len(gold_solution[0]))
            grades.addMessage('\tcorrect (reversed) solution length: %s' % len(gold_solution[1]))
            grades.addMessage('\tcorrect solution:\n%s' % wrap_solution(gold_solution[0]))
            grades.addMessage('\tcorrect (reversed) solution:\n%s' % wrap_solution(gold_solution[1]))
            return False

        if expanded > self.leewayFactor * gold_expanded and expanded > gold_expanded + 1:
            grades.addMessage('FAIL: %s' % self.path)
            grades.addMessage('Too many node expanded; are you expanding nodes twice?')
            grades.addMessage('\tstudent nodes expanded: %s' % expanded)
            grades.addMessage('')
            grades.addMessage('\tcorrect nodes expanded: %s (leewayFactor %s)' % (gold_expanded, self.leewayFactor))
            return False

        grades.addMessage('PASS: %s' % self.path)
        grades.addMessage('\tpacman layout:\t\t%s' % self.layoutName)
        grades.addMessage('\tsolution length: %s' % len(solution))
        grades.addMessage('\tnodes expanded:\t\t%s' % expanded)
        return True

    def writeSolution(self, moduleDict, filePath):
        search = moduleDict['search']
        searchAgents = moduleDict['searchAgents']
        # open file and write comments
        handle = open(filePath, 'w')
        handle.write('# This is the solution file for %s.\n' % self.path)
        handle.write('# This solution is designed to support both right-to-left\n')
        handle.write('# and left-to-right implementations.\n')
        handle.write(
            '# Number of nodes expanded must be with a factor of %s of the numbers below.\n' % self.leewayFactor)

        # write forward solution
        solution, expanded, error = self.getSolInfo(search, searchAgents)
        if error != None: raise Exception("Error in solution code: %s" % error)
        handle.write('solution: """\n%s\n"""\n' % wrap_solution(solution))
        handle.write('expanded_nodes: "%s"\n' % expanded)

        # write backward solution
        search.REVERSE_PUSH = not search.REVERSE_PUSH
        solution, expanded, error = self.getSolInfo(search, searchAgents)
        if error != None: raise Exception("Error in solution code: %s" % error)
        handle.write('rev_solution: """\n%s\n"""\n' % wrap_solution(solution))
        handle.write('rev_expanded_nodes: "%s"\n' % expanded)

        # clean up
        search.REVERSE_PUSH = not search.REVERSE_PUSH
        handle.close()
        return True


from game import Actions, Grid


def getStatesFromPath(start, path):
    "Returns the list of states visited along the path"
    vis = [start]
    curr = start
    for a in path:
        x, y = curr
        dx, dy = Actions.directionToVector(a)
        curr = (int(x + dx), int(y + dy))
        vis.append(curr)
    return vis


class CornerProblemTest(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(CornerProblemTest, self).__init__(question, testDict)
        self.layoutText = testDict['layout']
        self.layoutName = testDict['layoutName']

    def solution(self, search, searchAgents):
        lay = layout.Layout([l.strip() for l in self.layoutText.split('\n')])
        gameState = pacman.GameState()
        gameState.initialize(lay, 0)
        problem = searchAgents.CornersProblem(gameState)
        path = search.bfs(problem)

        gameState = pacman.GameState()
        gameState.initialize(lay, 0)
        visited = getStatesFromPath(gameState.getPacmanPosition(), path)
        top, right = gameState.getWalls().height - 2, gameState.getWalls().width - 2
        missedCorners = [p for p in ((1, 1), (1, top), (right, 1), (right, top)) if p not in visited]

        return path, missedCorners

    def execute(self, grades, moduleDict, solutionDict):
        search = moduleDict['search']
        searchAgents = moduleDict['searchAgents']
        gold_length = int(solutionDict['solution_length'])
        solution, missedCorners = self.solution(search, searchAgents)

        if type(solution) != type([]):
            grades.addMessage('FAIL: %s' % self.path)
            grades.addMessage('The result must be a list. (Instead, it is %s)' % type(solution))
            return False

        if len(missedCorners) != 0:
            grades.addMessage('FAIL: %s' % self.path)
            grades.addMessage('Corners missed: %s' % missedCorners)
            return False

        if len(solution) != gold_length:
            grades.addMessage('FAIL: %s' % self.path)
            grades.addMessage('Optimal solution not found.')
            grades.addMessage('\tstudent solution length:\n%s' % len(solution))
            grades.addMessage('')
            grades.addMessage('\tcorrect solution length:\n%s' % gold_length)
            return False

        grades.addMessage('PASS: %s' % self.path)
        grades.addMessage('\tpacman layout:\t\t%s' % self.layoutName)
        grades.addMessage('\tsolution length:\t\t%s' % len(solution))
        return True

    def writeSolution(self, moduleDict, filePath):
        search = moduleDict['search']
        searchAgents = moduleDict['searchAgents']
        # open file and write comments
        handle = open(filePath, 'w')
        handle.write('# This is the solution file for %s.\n' % self.path)

        print("Solving problem", self.layoutName)
        print(self.layoutText)

        path, _ = self.solution(search, searchAgents)
        length = len(path)
        print("Problem solved")

        handle.write('solution_length: "%s"\n' % length)
        handle.close()


# template = """class: "HeuristicTest"
#
# heuristic: "foodHeuristic"
# searchProblemClass: "FoodSearchProblem"
# layoutName: "Test %s"
# layout: \"\"\"
# %s
# \"\"\"
# """
#
# for i, (_, _, l) in enumerate(doneTests + foodTests):
#     f = open("food_heuristic_%s.test" % (i+1), "w")
#     f.write(template % (i+1, "\n".join(l)))
#     f.close()

class HeuristicTest(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(HeuristicTest, self).__init__(question, testDict)
        self.layoutText = testDict['layout']
        self.layoutName = testDict['layoutName']
        self.searchProblemClassName = testDict['searchProblemClass']
        self.heuristicName = testDict['heuristic']

    def setupProblem(self, searchAgents):
        lay = layout.Layout([l.strip() for l in self.layoutText.split('\n')])
        gameState = pacman.GameState()
        gameState.initialize(lay, 0)
        problemClass = getattr(searchAgents, self.searchProblemClassName)
        problem = problemClass(gameState)
        state = problem.getStartState()
        heuristic = getattr(searchAgents, self.heuristicName)

        return problem, state, heuristic

    def checkHeuristic(self, heuristic, problem, state, solutionCost):
        h0 = heuristic(state, problem)

        if solutionCost == 0:
            if h0 == 0:
                return True, ''
            else:
                return False, 'Heuristic failed H(goal) == 0 test'

        if h0 < 0:
            return False, 'Heuristic failed H >= 0 test'
        if not h0 > 0:
            return False, 'Heuristic failed non-triviality test'
        if not h0 <= solutionCost:
            return False, 'Heuristic failed admissibility test'

        for succ, action, stepCost in problem.getSuccessors(state):
            h1 = heuristic(succ, problem)
            if h1 < 0: return False, 'Heuristic failed H >= 0 test'
            if h0 - h1 > stepCost: return False, 'Heuristic failed consistency test'

        return True, ''

    def execute(self, grades, moduleDict, solutionDict):
        search = moduleDict['search']
        searchAgents = moduleDict['searchAgents']
        solutionCost = int(solutionDict['solution_cost'])
        problem, state, heuristic = self.setupProblem(searchAgents)

        passed, message = self.checkHeuristic(heuristic, problem, state, solutionCost)

        if not passed:
            grades.addMessage('FAIL: %s' % self.path)
            grades.addMessage('%s' % message)
            return False
        else:
            grades.addMessage('PASS: %s' % self.path)
            return True

    def writeSolution(self, moduleDict, filePath):
        search = moduleDict['search']
        searchAgents = moduleDict['searchAgents']
        # open file and write comments
        handle = open(filePath, 'w')
        handle.write('# This is the solution file for %s.\n' % self.path)

        print("Solving problem", self.layoutName, self.heuristicName)
        print(self.layoutText)
        problem, _, heuristic = self.setupProblem(searchAgents)
        path = search.astar(problem, heuristic)
        cost = problem.getCostOfActions(path)
        print("Problem solved")

        handle.write('solution_cost: "%s"\n' % cost)
        handle.close()
        return True


# class MAPFTest(testClasses.TestCase):

#     def __init__(self, question, testDict):
#         super(MAPFTest, self).__init__(question, testDict)
#         self.layoutText = testDict['layout']
#         self.ans = testDict['ans']
#         self.timeout = DEFAULT_TIMEOUT

#     def setupProblem(self, search):
#         lay = layout.Layout([l.strip() for l in self.layoutText.split('\n')])
#         gameState = pacman.GameState()
#         gameState.initialize(lay, 0)
#         problemClass = getattr(search, "MAPFProblem")
#         problem = problemClass(gameState)
#         state = problem.getStartState()

#         return problem, state

#     def execute(self, grades, moduleDict, solutionDict):
#         search = moduleDict['search']
#         problem, _ = self.setupProblem(search)

#         search_sol = moduleDict['search_sol']
#         problem_sol, _ = self.setupProblem(search_sol)

#         try:
#             start_time = datetime.datetime.now()
#             sol = func_timeout(self.timeout, search_sol.bfs, args=(problem,))
#             execute_time = datetime.datetime.now() - start_time
#             grades.addMessage('Search time: %s' % execute_time)
#         except FunctionTimedOut:
#             # traceback.print_exc()
#             grades.addMessage('FAIL: timeout after %ds' % self.timeout)
#             grades.addMessage('FAIL: %s' % self.path)
#             return False
#         except:
#             traceback.print_exc()
#             grades.addMessage('FAIL: %s' % self.path)
#             return False
#         try:
#             if not checkSolutionVerify(problem_sol, sol, problem):
#                 grades.addMessage('FAIL: %s' % self.path)
#                 grades.addMessage('\tReturned path is not a solution.')
#                 return False
#         except ValueError as e:
#             grades.addMessage(('FAIL: %s' % self.path))
#             grades.addMessage('\tYour formulation might be incorrect: ' + repr(e))
#             return False

#         answer = len(sol)
#         if answer == int(self.ans):
#             grades.addMessage('PASS: %s' % self.path)
#         else:
#             grades.addMessage('FAIL: %s' % self.path)
#             grades.addMessage('\tReturned path is not an optimal solution.')
#             return False

#         return True

#     def writeSolution(self, moduleDict, filePath):
#         handle = open(filePath, 'w')
#         handle.write('# This is the solution file for %s.\n' % self.path)
#         handle.write('# File intentionally blank.\n')
#         handle.close()
#         return True


class cbsTest(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(cbsTest, self).__init__(question, testDict)
        self.layoutText = testDict['layout']
        self.ans = testDict['ans']
        self.timeout = DEFAULT_TIMEOUT

    def setupProblem(self, search_problems):
        lay = layout.Layout([l.strip() for l in self.layoutText.split('\n')])
        gameState = pacman.GameState()
        gameState.initialize(lay, 0)
        problemClass = getattr(search_problems, "MAPFProblem")
        problem = problemClass(gameState)
        state = problem.getStartState()

        return problem, state

    def execute(self, grades, moduleDict, solutionDict):
        # search_sol = moduleDict['search_sol']
        search_problems = moduleDict['searchProblems']
        problem_sol, _ = self.setupProblem(search_problems)

        search = moduleDict['search']


        try:
            start_time = datetime.datetime.now()
            sol = func_timeout(self.timeout, search.cbs, args=(problem_sol,))
            execute_time = datetime.datetime.now() - start_time
            grades.addMessage('Search time: %s' % execute_time)
        except FunctionTimedOut:
            # traceback.print_exc()
            grades.addMessage('FAIL: timeout after %ds' % self.timeout)
            grades.addMessage('FAIL: %s' % self.path)
            return False
        except:
            traceback.print_exc()
            grades.addMessage('FAIL: %s' % self.path)
            return False

        if sol == None:
            grades.addMessage('FAIL: %s' % self.path)
            grades.addMessage('\tThe path returned is None.')
            return False
        elif not checkTrajs(sol, problem_sol):
            grades.addMessage('FAIL: %s' % self.path)
            grades.addMessage('\tReturned path is not a solution.')
            return False


        answer = sum([len(sol[p]) for p in sol])
        if answer == int(self.ans):
            grades.addMessage('PASS: %s' % self.path)
        else:
            grades.addMessage('FAIL: %s' % self.path)
            grades.addMessage('\tReturned path is not an optimal solution.')
            grades.addMessage('\tstudent solution length: %s' % answer)
            grades.addMessage('\tcorrect solution length: %s' % self.ans)
            return False

        return True

    def writeSolution(self, moduleDict, filePath):
        handle = open(filePath, 'w')
        handle.write('# This is the solution file for %s.\n' % self.path)
        handle.write('# File intentionally blank.\n')
        handle.close()
        return True


class HeuristicGrade(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(HeuristicGrade, self).__init__(question, testDict)
        self.layoutText = testDict['layout']
        self.layoutName = testDict['layoutName']
        self.searchProblemClassName = testDict['searchProblemClass']
        self.heuristicName = testDict['heuristic']
        self.basePoints = int(testDict['basePoints'])
        self.thresholds = [int(t) for t in testDict['gradingThresholds'].split()]
        self.timeout = DEFAULT_TIMEOUT

    def setupProblem(self, search, search_problems):
        lay = layout.Layout([l.strip() for l in self.layoutText.split('\n')])
        gameState = pacman.GameState()
        gameState.initialize(lay, 0)
        problemClass = getattr(search_problems, self.searchProblemClassName)
        problem = problemClass(gameState)
        state = problem.getStartState()
        heuristic = getattr(search, self.heuristicName)

        return problem, state, heuristic

    def execute(self, grades, moduleDict, solutionDict):
        search = moduleDict['search']
        test_search = moduleDict['testSearch']
        search_problems = moduleDict['searchProblems']
        problem, _, heuristic = self.setupProblem(search, search_problems)

        try:
            start_time = datetime.datetime.now()
            path = func_timeout(self.timeout, test_search.astar, args=(problem, heuristic))
            execute_time = datetime.datetime.now() - start_time
            grades.addMessage('Search time: %s' % execute_time)
        except FunctionTimedOut:
            # traceback.print_exc()
            grades.addMessage('FAIL: timeout after %ds' % self.timeout)
            grades.addMessage('FAIL: %s' % self.path)
            return False
        except:
            traceback.print_exc()
            grades.addMessage('FAIL: %s' % self.path)
            return False

        expanded = problem._expanded

        if path == None:
            grades.addMessage('FAIL: %s' % self.path)
            grades.addMessage('\tThe path returned is None.')
            # grades.addMessage('\texpanded nodes: %s' % expanded)
            return False

        elif not checkSolution(problem, path):
            grades.addMessage('FAIL: %s' % self.path)
            grades.addMessage('\tReturned path is not a solution.')
            grades.addMessage('\texpanded nodes: %s' % expanded)
            return False

        grades.addPoints(self.basePoints/5)
        points = 0
        for threshold in self.thresholds:
            if expanded <= threshold:
                points += 1


        if points >= len(self.thresholds):
            grades.addMessage('PASS: %s' % self.path)
        else:
            grades.addMessage('FAIL: %s' % self.path)

        grades.addPoints(points/5)
        grades.addMessage('\texpanded nodes: %s' % expanded)
        grades.addMessage('\tthresholds: %s' % self.thresholds)

        return True

    def writeSolution(self, moduleDict, filePath):
        handle = open(filePath, 'w')
        handle.write('# This is the solution file for %s.\n' % self.path)
        handle.write('# File intentionally blank.\n')
        handle.close()
        return True


# template = """class: "ClosestDotTest"
#
# layoutName: "Test %s"
# layout: \"\"\"
# %s
# \"\"\"
# """
#
# for i, (_, _, l) in enumerate(foodTests):
#     f = open("closest_dot_%s.test" % (i+1), "w")
#     f.write(template % (i+1, "\n".join(l)))
#     f.close()

class ClosestDotTest(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(ClosestDotTest, self).__init__(question, testDict)
        self.layoutText = testDict['layout']
        self.layoutName = testDict['layoutName']

    def solution(self, searchAgents):
        lay = layout.Layout([l.strip() for l in self.layoutText.split('\n')])
        gameState = pacman.GameState()
        gameState.initialize(lay, 0)
        path = searchAgents.ClosestDotSearchAgent().findPathToClosestDot(gameState)
        return path

    def execute(self, grades, moduleDict, solutionDict):
        search = moduleDict['search']
        searchAgents = moduleDict['searchAgents']
        gold_length = int(solutionDict['solution_length'])
        solution = self.solution(searchAgents)

        if type(solution) != type([]):
            grades.addMessage('FAIL: %s' % self.path)
            grades.addMessage('\tThe result must be a list. (Instead, it is %s)' % type(solution))
            return False

        if len(solution) != gold_length:
            grades.addMessage('FAIL: %s' % self.path)
            grades.addMessage('Closest dot not found.')
            grades.addMessage('\tstudent solution length:\n%s' % len(solution))
            grades.addMessage('')
            grades.addMessage('\tcorrect solution length:\n%s' % gold_length)
            return False

        grades.addMessage('PASS: %s' % self.path)
        grades.addMessage('\tpacman layout:\t\t%s' % self.layoutName)
        grades.addMessage('\tsolution length:\t\t%s' % len(solution))
        return True

    def writeSolution(self, moduleDict, filePath):
        search = moduleDict['search']
        searchAgents = moduleDict['searchAgents']
        # open file and write comments
        handle = open(filePath, 'w')
        handle.write('# This is the solution file for %s.\n' % self.path)

        print("Solving problem", self.layoutName)
        print(self.layoutText)

        length = len(self.solution(searchAgents))
        print("Problem solved")

        handle.write('solution_length: "%s"\n' % length)
        handle.close()
        return True


class CornerHeuristicSanity(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(CornerHeuristicSanity, self).__init__(question, testDict)
        self.layout_text = testDict['layout']

    def execute(self, grades, moduleDict, solutionDict):
        search = moduleDict['search']
        searchAgents = moduleDict['searchAgents']
        game_state = pacman.GameState()
        lay = layout.Layout([l.strip() for l in self.layout_text.split('\n')])
        game_state.initialize(lay, 0)
        problem = searchAgents.CornersProblem(game_state)
        start_state = problem.getStartState()
        h0 = searchAgents.cornersHeuristic(start_state, problem)
        succs = problem.getSuccessors(start_state)
        # cornerConsistencyA
        for succ in succs:
            h1 = searchAgents.cornersHeuristic(succ[0], problem)
            if h0 - h1 > 1:
                grades.addMessage('FAIL: inconsistent heuristic')
                return False
        heuristic_cost = searchAgents.cornersHeuristic(start_state, problem)
        true_cost = float(solutionDict['cost'])
        # cornerNontrivial
        if heuristic_cost == 0:
            grades.addMessage('FAIL: must use non-trivial heuristic')
            return False
        # cornerAdmissible
        if heuristic_cost > true_cost:
            grades.addMessage('FAIL: Inadmissible heuristic')
            return False
        path = solutionDict['path'].split()
        states = followPath(path, problem)
        heuristics = []
        for state in states:
            heuristics.append(searchAgents.cornersHeuristic(state, problem))
        for i in range(0, len(heuristics) - 1):
            h0 = heuristics[i]
            h1 = heuristics[i + 1]
            # cornerConsistencyB
            if h0 - h1 > 1:
                grades.addMessage('FAIL: inconsistent heuristic')
                return False
            # cornerPosH
            if h0 < 0 or h1 < 0:
                grades.addMessage('FAIL: non-positive heuristic')
                return False
        # cornerGoalH
        if heuristics[len(heuristics) - 1] != 0:
            grades.addMessage('FAIL: heuristic non-zero at goal')
            return False
        grades.addMessage('PASS: heuristic value less than true cost at start state')
        return True

    def writeSolution(self, moduleDict, filePath):
        search = moduleDict['search']
        searchAgents = moduleDict['searchAgents']
        # write comment
        handle = open(filePath, 'w')
        handle.write('# In order for a heuristic to be admissible, the value\n')
        handle.write('# of the heuristic must be less at each state than the\n')
        handle.write('# true cost of the optimal path from that state to a goal.\n')

        # solve problem and write solution
        lay = layout.Layout([l.strip() for l in self.layout_text.split('\n')])
        start_state = pacman.GameState()
        start_state.initialize(lay, 0)
        problem = searchAgents.CornersProblem(start_state)
        solution = search.astar(problem, searchAgents.cornersHeuristic)
        handle.write('cost: "%d"\n' % len(solution))
        handle.write('path: """\n%s\n"""\n' % wrap_solution(solution))
        handle.close()
        return True


class CornerHeuristicPacman(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(CornerHeuristicPacman, self).__init__(question, testDict)
        self.layout_text = testDict['layout']

    def execute(self, grades, moduleDict, solutionDict):
        search = moduleDict['search']
        searchAgents = moduleDict['searchAgents']
        total = 0
        true_cost = float(solutionDict['cost'])
        thresholds = [int(x) for x in solutionDict['thresholds'].split()]
        game_state = pacman.GameState()
        lay = layout.Layout([l.strip() for l in self.layout_text.split('\n')])
        game_state.initialize(lay, 0)
        problem = searchAgents.CornersProblem(game_state)
        start_state = problem.getStartState()
        if searchAgents.cornersHeuristic(start_state, problem) > true_cost:
            grades.addMessage('FAIL: Inadmissible heuristic')
            return False
        path = search.astar(problem, searchAgents.cornersHeuristic)
        print("path:", path)
        print("path length:", len(path))
        cost = problem.getCostOfActions(path)
        if cost > true_cost:
            grades.addMessage('FAIL: Inconsistent heuristic')
            return False
        expanded = problem._expanded
        points = 0
        for threshold in thresholds:
            if expanded <= threshold:
                points += 1
        grades.addPoints(points)
        if points >= len(thresholds):
            grades.addMessage('PASS: Heuristic resulted in expansion of %d nodes' % expanded)
        else:
            grades.addMessage('FAIL: Heuristic resulted in expansion of %d nodes' % expanded)
        return True

    def writeSolution(self, moduleDict, filePath):
        search = moduleDict['search']
        searchAgents = moduleDict['searchAgents']
        # write comment
        handle = open(filePath, 'w')
        handle.write('# This solution file specifies the length of the optimal path\n')
        handle.write('# as well as the thresholds on number of nodes expanded to be\n')
        handle.write('# used in scoring.\n')

        # solve problem and write solution
        lay = layout.Layout([l.strip() for l in self.layout_text.split('\n')])
        start_state = pacman.GameState()
        start_state.initialize(lay, 0)
        problem = searchAgents.CornersProblem(start_state)
        solution = search.astar(problem, searchAgents.cornersHeuristic)
        handle.write('cost: "%d"\n' % len(solution))
        handle.write('path: """\n%s\n"""\n' % wrap_solution(solution))
        handle.write('thresholds: "2000 1600 1200"\n')
        handle.close()
        return True


# ### The following is changed for 2022S2 BAE*
# it looks like this works on general search problem as well
class PacmanBidirectionalSearchTest(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(PacmanBidirectionalSearchTest, self).__init__(question, testDict)
        self.layout_text = testDict['layout']
        self.alg = testDict['algorithm']
        self.layoutName = testDict['layoutName']

        # TODO: sensible to have defaults like this?
        self.leewayFactor = float(testDict.get('leewayFactor', '1'))
        self.costFn = eval(testDict.get('costFn', 'None'))
        self.searchProblemClassName = testDict.get('searchProblemClass', 'BidirectionalPositionSearchProblem')
        self.heuristicName = testDict.get('heuristic', None)
        self.backwardsHeuristicName = testDict.get('backwardsHeuristic', None)
        self.timeout = int(testDict.get('timeout', DEFAULT_TIMEOUT))

    def getSolInfo(self, search, searchAgents):
        alg = getattr(search, self.alg)
        lay = layout.Layout([l.strip() for l in self.layout_text.split('\n')])
        start_state = pacman.GameState()
        start_state.initialize(lay, 0)

        problemClass = getattr(searchAgents, self.searchProblemClassName)
        problemOptions = {}
        if self.costFn != None:
            problemOptions['costFn'] = self.costFn
        problem = problemClass(start_state, **problemOptions)
        heuristic = getattr(searchAgents, self.heuristicName) if self.heuristicName != None else None
        backwardsHeuristic = getattr(searchAgents,
                                     self.backwardsHeuristicName) if self.backwardsHeuristicName != None else None

        goal = lay.food.asList()[0]
        problem.goal = goal

        if heuristic != None and backwardsHeuristic != None:
            solution = alg(problem, heuristic, backwardsHeuristic)
        elif heuristic != None:
            solution = alg(problem, heuristic)
        else:
            solution = alg(problem)

        if type(solution) != type([]):
            return None, None, 'The result of %s must be a list. (Instead, it is %s)' % (self.alg, type(solution))

        from game import Directions
        dirs = Directions.LEFT.keys()
        if [el in dirs for el in solution].count(False) != 0:
            return None, None, 'Output of %s must be a list of actions from game.Directions' % self.alg

        expanded = problem._expanded
        return solution, expanded, None

    def execute(self, grades, moduleDict, solutionDict):

        search = moduleDict['search']
        searchAgents = moduleDict['searchAgents']

        # gathering all solutions
        gold_solution = {}
        gold_solution_length = {}

        for solution_name in ['solution', 'rev_solution', 'alt_solution', 'alt_rev_solution']:

            solution_str = solutionDict.get(solution_name, None)
            if not solution_str == None:
                solution = str.split(solution_str)
                gold_solution.update({solution_name: solution})

            solution_length = solutionDict.get(solution_name + '_length', None)
            if not solution_length == None:
                gold_solution_length.update({solution_name + '_length': int(solution_length)})

        gold_expanded = {}
        for expansion_name in ['expanded_nodes', 'rev_expanded_nodes', 'alt_rev_solution', 'alt_rev_expanded_nodes']:
            expanded_nodes = solutionDict.get(expansion_name, None)
            if not expanded_nodes == None:
                gold_expanded.update({expansion_name: int(expanded_nodes)})

        solution_length = solutionDict.get("solution_length", None)
        if not solution_length == None:
            gold_solution_length.update({"solution_length": int(solution_length)})

        try:

            start_time = datetime.datetime.now()
            solution, expanded, error = func_timeout(self.timeout, self.getSolInfo, args=(search, searchAgents))
            execute_time = datetime.datetime.now() - start_time
            grades.addMessage('Search time: %s' % execute_time)
            # solution, expanded, error = util.TimeoutFunction(self.getSolInfo,5)(search,searchAgents) # Call the question's function
            # TimeoutFunction(getattr(gradingModule, q),1200)(self) # Call the question's function
        except FunctionTimedOut:
            # traceback.print_exc()
            grades.addMessage('FAIL: timeout after %ds' % self.timeout)
            grades.addMessage('FAIL: %s' % self.path)
            return False
        except:
            traceback.print_exc()
            grades.addMessage('FAIL: %s' % self.path)
            return False

        # if length provided in solution, we check length
        if not gold_solution_length == {}:
            if not int(len(solution)) in gold_solution_length.values():
                grades.addMessage('FAIL: %s' % self.path)
                grades.addMessage('Solution length is not optimal')
                grades.addMessage('\tstudent solution length: %d' % len(solution))
                grades.addMessage('')
                grades.addMessage('\tcorrect solutions length: %s' % str(gold_solution_length))
                return False

        if not gold_solution == {}:
            if solution not in gold_solution.values():
                grades.addMessage('FAIL: %s' % self.path)
                grades.addMessage('Solution not correct.')
                grades.addMessage('\tstudent solution length: %s' % len(solution))
                grades.addMessage('\tstudent solution:\n%s' % wrap_solution(solution))
                grades.addMessage('')

                for i, g_solution in gold_solution.items():
                    grades.addMessage('\tcorrect solution %s:' % i)
                    grades.addMessage('\tsolution length: %s' % len(g_solution))
                    grades.addMessage('\tsolution:\n%s' % wrap_solution(g_solution))
                return False

        if not gold_expanded == {}:
            expansion = False
            for gold_expansion in gold_expanded.values():
                if expanded <= math.ceil(self.leewayFactor * gold_expansion) and expanded >= math.ceil(
                        gold_expansion / self.leewayFactor):
                    expansion = True
            if not expansion:
                grades.addMessage('FAIL: %s' % self.path)
                grades.addMessage('Wrong node expansion numbers.')
                grades.addMessage('\tstudent nodes expanded: %s' % expanded)
                grades.addMessage('')
                grades.addMessage('\tcorrect nodes expanded: %s (leewayFactor %s)' % (gold_expanded, self.leewayFactor))
                return False

        grades.addMessage('PASS: %s' % self.path)
        grades.addMessage('\tpacman layout:\t\t%s' % self.layoutName)
        grades.addMessage('\tsolution length: %s' % len(solution))
        grades.addMessage('\tnodes expanded:\t\t%s' % expanded)
        return True
