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
import math

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    corners = 0
    explored = []
    frontier = []
    start = problem.getStartState()
    frontier = frontier + [start]
    memory_explored=0
    memory_frontier=0

    while True:
        if len(frontier) == 0:
            return []

        currentState = frontier.pop(0)

        #checking memory use:
        if (memory_explored < len(explored) ):
            memory_explored = len(explored)
        if (memory_frontier < len(frontier) ):
            memory_frontier = len(frontier)

        if problem.isGoalState(currentState):
            sum= memory_explored + memory_frontier
            print("maximum memory use: ", sum)
            return currentState.actions

        if (problem.cornerCount > corners):
            corners = corners + 1
            frontier = []
            explored = []
            #print currentState.position

        explored = explored + [currentState]

        successors = problem.getSuccessors(currentState)
        for successor in successors:
            explore = True
            for frontierNode in frontier:
                if successor[0].position == frontierNode.position:
                    explore = False
            if explore == True:
                for q in explored:
                    if successor[0].position == q.position:
                        explore = False
            if explore == True:
                successor[0].actions = currentState.actions + [successor[1]]
                frontier = [successor[0]] + frontier

def breadthFirstSearch(problem):
    corners = 0
    explored = []
    frontier = []
    start = problem.getStartState()
    frontier = frontier + [start]
    memory_explored=0
    memory_frontier=0

    while True:
        if len(frontier) == 0:
            return []

        currentState = frontier.pop(0)

        #checking memory use:
        if (memory_explored < len(explored) ):
            memory_explored = len(explored)
        if (memory_frontier < len(frontier) ):
            memory_frontier = len(frontier)

        if problem.isGoalState(currentState):
            sum = memory_explored + memory_frontier
            print("maximum memory use: ", sum)
            return currentState.actions

        if (problem.cornerCount > corners):
            currentState = frontier.pop()   #Differ from DFS. (does not excist)
            corners = corners + 1
            frontier = []
            explored = []

        explored = explored + [currentState]

        successors = problem.getSuccessors(currentState)
        for successor in successors:    #"for each action in problem.actions"
            explore = True
            for frontierNode in frontier:
                if successor[0].position == frontierNode.position:
                    explore = False
            if explore == True:
                for q in explored:
                    if successor[0].position == q.position:
                        explore = False
            if explore == True:
                successor[0].actions = currentState.actions + [successor[1]]
                frontier = frontier + [successor[0]]   #other way around in dfs.

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** DO NOT IMPLEMENT ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    "*** DO NOT IMPLEMENT ***"
    return 0

def cornersHeuristic(state, problem=None):
    h=99999
    x,y = state.position
    #for corner in problem.corners
    for c in problem.corners:
        dx,dy = c
        h_new = math.pow((x-dx),2) + math.pow((y-dy),2)
        h_new = math.sqrt(h_new)
        if (h_new < h):
            h = h_new
    return h

def aStarSearch(problem, heuristic=nullHeuristic):
    corners = 0
    explored = []
    frontier = dict()
    start = problem.getStartState()
    frontier[start]=([],0)  #(path,cost) no path in first state
    memory_explored=0
    memory_frontier=0
    memory_sortedfrontier=0


    while not len(frontier)==0:
        orderedStates=sorted(frontier, key=lambda i: frontier[i][1]) #orderstates is only the nodes in a list ordered by it's paths.
        currentState=orderedStates[0]  #takes the best one

        temp = frontier.get(currentState)  #and pics out the path of that one  (path=frontier[currentState][0])
        path = temp[0]
        cost = temp[1]

        #checking memory use:
        if (memory_explored < len(explored) ):
            memory_explored = len(explored)
        if (memory_frontier < len(frontier) ):
            memory_frontier = len(frontier)
        if (memory_sortedfrontier < len(orderedStates)):
            memory_explored = len(orderedStates)

        if problem.isGoalState(currentState):
                sum= memory_explored + memory_frontier + memory_sortedfrontier
                print("maximum memory use: ", sum)
                return path

        del frontier[currentState]   #make it work as pop.

        explored = explored + [currentState]
        successors = problem.getSuccessors(currentState)

        for successor in successors:
            new_Path = path + [successor[1]] #creates a list with the actions.
            new_cost = problem.getCostOfActions(new_Path) + cornersHeuristic(successor[0], problem) # g+h but h is only 0, same as nullHeuristic
            explore = True

            for q in explored:
                if successor[0].position == q.position:
                    explore = False
            if explore == True:

                for frontierNode in frontier:
                    if successor[0].position == frontierNode.position:
                        explore = False
                        #if successor already are in frontier and have a higher cost then the new_cost. change it to new better cost and path.
                        if(successor[2] > new_cost):
                            #Never gets here?
                            #print("better cost found", successor[1]," >", new_cost)
                            del frontier[successor[0]]
                            frontier[successor[0]]=(new_Path,new_cost)
            if explore == True:
                frontier[successor[0]]=(new_Path,new_cost)

            if (problem.cornerCount > corners):
                if(successor[0].position in problem.corners):
                    corners = corners + 1
                    frontier = dict()
                    explored = []
                    frontier[successor[0]]=(new_Path,new_cost)
                    break
    return []   #when jumping out of while-loop.

class Node(object):
    def __init__(self, state=None):
        self.state=state
        self.solution=[]

def childNode(parent, action):
    theState = action[0].position  ## coordinates (5,1)
    path = action[1]            ##North, south ..

    child = Node()
    child.state = theState
    child.solution = list(parent.solution)
    child.solution.append(path)

    return child

def recursiveDLS(node, problem, limit, explored, foundCorner, memory_use):

    #checking memory usage
    #print(memory_use, "<", len(node.solution) + len(explored), (memory_use < len(node.solution) + len(explored))
    if(memory_use < len(node.solution) + len(explored)):
        memory_use = len(node.solution) + len(explored)

    if problem.isGoalStateIterative(node.state) and len(foundCorner) == 4:
        print("maximum memory use: ", memory_use)
        return node.solution

    elif limit == 0:
        return "cutoff"
    else:
        cutoff_occured = False
        explored.add(node.state)
        actions = problem.getSuccessorActions(node.state)
        for action in actions:
            child = childNode(node ,action)
            if (problem.cornerCount > len(foundCorner)) and (child.state in problem.corners) and (child.state not in foundCorner):
                explored = set()
                foundCorner = foundCorner + [child.state]

            if not(child.state in explored):
                result = recursiveDLS(child,problem,limit-1,explored, foundCorner, memory_use)
                if result == "cutoff":
                    cutoff_occured = True
                elif result != "failure":
                    return result
        if cutoff_occured:
            return "cutoff"
        else:
            return "failure"

def depthLimitedSearch(problem, limit):
    start = problem.getStartState()
    startnode = Node(state=start.position)
    explored = set()
    foundCorner=[]
    memory_use = 0
    return recursiveDLS(startnode,problem,limit,explored, foundCorner, memory_use)

def iDeepeningSearch(problem):
    """ Iterative deepening search.
    Iterative DFS to achieve a result similar to breadthFirstSearch without taking up as much space. """
    depth = 0
    while True:
        result = depthLimitedSearch(problem, depth)
        if result != "cutoff":
            if result == "failure":
                print(" ............The test failed............")
                return []
            print("Depth reached:", depth)
            return result
        depth = depth + 1


def bidirectional_search(problem):

    frontIni = util.Queue()
    frontGoal = util.Queue()

    explo1 = list()
    explo2 = list()

    frontIni.push((problem.getStartState(), list()))
    frontGoal.push((problem.goal, list()))

    goal2 = problem.getStartState()

    #se aplciara bfs a ambas fronteras y cuando tengau un nodo en comun es porque
    #se encontro un camino.

    while not frontIni.isEmpty():
        nodo1 = frontIni.pop()
        nodo2 = frontGoal.pop()

        for data in problem.getSuccessors(nodo1[0]):

            if not data[0] in explo1:
                if problem.isGoalState(data[0]):
                    return nodo1[1] + [data[1]]

                frontIni.push((data[0], nodo1[1] + [data[1]]))
                explo1.append(data[0])

        for data in problem.getSuccessors(nodo2[0]):
            if not data[0] in explo2:
                if data[0] == goal2:
                    return [data[1]] + nodo2[1][::-1]
                frontGoal.push((data[0], nodo2[1] + [data[1]]))
                explo2.append(data[0])

        #verificacion si las fronteras tienen un estado comun
        for node in frontIni.list:
            for elem in frontGoal.list:
                if node[0] == elem[0]:
                    act1 = node[1]
                    act2 = elem[1]
                    act2 = act2[::-1]#inversion de la lista de acciones act2

                    for i in range(len(act2)):#reemplazamos los valores
                        if act2[i] == "North": act2[i] = "South"
                        elif act2[i] == "South": act2[i] = "North"
                        elif act2[i] == "West": act2[i] = "East"
                        elif act2[i] == "East": act2[i] = "West"
                    return act1 + act2
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
astar = aStarSearch
ids = iDeepeningSearch
bs  = bidirectional_search
