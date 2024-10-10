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

def depthFirstSearch(problem: SearchProblem):
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
    "*** YOUR CODE HERE ***"
    print("Start:", problem.getStartState())
    print("Is Start the goal?", problem.isGoalState(problem.getStartState()))
   # print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    fringe = util.Stack()
    directions = util.Stack()
    path = []
    expanded = []
    data = {}

    expanded.append(problem.getStartState())
    

    # push initial nodes into stack and dataset
    for node in problem.getSuccessors(problem.getStartState()):
        fringe.push(node)
        data[node[0]] = (problem.getStartState(), node[1])

    while not fringe.isEmpty():
        node = fringe.pop()
        expanded.append(node[0])

        # if goal found
        if (problem.isGoalState(node[0]) == True):
            state = node[0]
            
            # rebuild path from goal using data set
            while state != problem.getStartState():
                #print(state)
                #print("goal found: rebuilding path")
                directions.push(data[state][1])
                state = data[state][0]
            while directions.isEmpty() != True:
               # print("building path")
                path.append(directions.pop())
            return path
        
        # if children exist
        for child in problem.getSuccessors(node[0]):

            # if child has not been searched
            if child[0] not in expanded:
                fringe.push(child)
                data[child[0]] = (node[0], child[1])



def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # should have just stored path in fringe?
    print("Start:", problem.getStartState())
    print("Is Start the goal?", problem.isGoalState(problem.getStartState()))
   # print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    fringe = util.Queue()
    directions = util.Stack()
    path = []
    expanded = []
    data = {}


    expanded.append(problem.getStartState())
    

    # push initial nodes into stack and dataset
    for node in problem.getSuccessors(problem.getStartState()):
        fringe.push(node)
        data[node[0]] = (problem.getStartState(), node[1])
        expanded.append(node[0])

    while not fringe.isEmpty():
        node = fringe.pop()
        #expanded.append(node[0])

        # if goal found
        if (problem.isGoalState(node[0]) == True):
            state = node[0]
            
            # rebuild path from goal using data set
            while state != problem.getStartState():
                #print(state)
                #print("goal found: rebuilding path")
                directions.push(data[state][1])
                state = data[state][0]
            while directions.isEmpty() != True:
               # print("building path")
                path.append(directions.pop())
            return path
        
        # if children exist
        for child in problem.getSuccessors(node[0]):

            # if child has not been searched
            if child[0] not in expanded:
                fringe.push(child)
                data[child[0]] = (node[0], child[1])
                expanded.append(child[0])


def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # Need to change the code to store all paths
    print("Start:", problem.getStartState())
    print("Is Start the goal?", problem.isGoalState(problem.getStartState()))
    #print("Start's successors:", problem.getSuccessors(problem.getStartState()))


    fringe = util.PriorityQueue()
    closed = []
    directions = {}

    # push root into fringe
    fringe.push(problem.getStartState(), 0)
    closed.append(problem.getStartState())
    directions[problem.getStartState()] = [[], 0]

    # loop over fringe
    while fringe.isEmpty() is not True:


        # pop out node
        node = fringe.pop()

        # if node goal
        if problem.isGoalState(node) == True:

            # return the path
            return directions[node][0]

        # for child of node
        for child in problem.getSuccessors(node):

            # if child not in closed
            if child[0] not in closed:

                # calculate cost
                cost = directions[node][1] + child[2]

                # calculate path
                path = directions[node][0].copy()
                path.append(child[1])

                # add child to fringe and closed and add data to directions
                fringe.update(child[0], cost)
                closed.append(child[0])
                directions[child[0]] = [path, cost]

            # if child in closed
            else:

                # calculate cost
                cost = directions[node][1] + child[2]
                # check if path is cheaper than current path 
                if cost < directions[child[0]][1]:

                     # calculate path
                    path = directions[node][0].copy()
                    path.append(child[1])

                    # update fringe and directions
                    fringe.update(child[0], cost)
                    directions[child[0]] = [path, cost]


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # Need to change the code to store all paths
    print("Start:", problem.getStartState())
    print("Is Start the goal?", problem.isGoalState(problem.getStartState()))
    #print("Start's successors:", problem.getSuccessors(problem.getStartState()))


    fringe = util.PriorityQueue()
    closed = []
    directions = {}

    # push root into fringe
    fringe.push(problem.getStartState(), 0)
    closed.append(problem.getStartState())
    directions[problem.getStartState()] = [[], 0, 0]

    # loop over fringe
    while fringe.isEmpty() is not True:


        # pop out node
        node = fringe.pop()

        # if node goal
        if problem.isGoalState(node) == True:

            # return the path
            return directions[node][0]

        # for child of node
        for child in problem.getSuccessors(node):

            # if child not in closed
            if child[0] not in closed:

                # calculate cost
                uniform = directions[node][2] + child[2]
                greedy = heuristic(child[0], problem)

                cost = uniform + greedy
                #print(child[0], cost)

                # calculate path
                path = directions[node][0].copy()
                path.append(child[1])

                # add child to fringe and closed and add data to directions
                fringe.update(child[0], cost)
                closed.append(child[0])
                directions[child[0]] = [path, cost, uniform]

            # if child in closed
            else:

                # calculate cost
                uniform = directions[node][2] + child[2]
                greedy = heuristic(child[0], problem)

                cost = uniform + greedy
                #print(child[0], cost)

                # check if path is cheaper than current path 
                if cost < directions[child[0]][1]:

                     # calculate path
                    path = directions[node][0].copy()
                    path.append(child[1])

                    # update fringe and directions
                    fringe.update(child[0], cost)
                    directions[child[0]] = [path, cost, uniform]


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
