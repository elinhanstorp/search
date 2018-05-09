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

    while True:
        if len(frontier) == 0:
            return []

        currentState = frontier.pop(0)

        if problem.isGoalState(currentState):
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

    while True:
        if len(frontier) == 0:
            return []

        currentState = frontier.pop(0)

        if problem.isGoalState(currentState):
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

    while not len(frontier)==0:
        orderedStates=sorted(frontier, key=lambda i: frontier[i][1]) #orderstates is only the nodes in a list ordered by it's paths.
        currentState=orderedStates[0]  #takes the best one

        temp = frontier.get(currentState)  #and pics out the path of that one  (path=frontier[currentState][0])
        path = temp[0]
        cost = temp[1]

        if problem.isGoalState(currentState):
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

def recursiveDLS(node, problem, limit, explored, foundCorner):

    if problem.isGoalStateIterative(node.state) and len(foundCorner) == 4:
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
                result = recursiveDLS(child,problem,limit-1,explored, foundCorner)
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
    return recursiveDLS(startnode,problem,limit,explored, foundCorner)

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

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
astar = aStarSearch
ids = iDeepeningSearch
