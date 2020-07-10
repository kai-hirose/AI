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
    visited=[]
    frontier=util.Stack()
    frontier.push((problem.getStartState(),list()))
    while(not frontier.isEmpty()):
        newState, path = frontier.pop()
        visited.append(newState)
        if problem.isGoalState(newState):
            return path
        for childstate, action, cost in problem.getSuccessors(newState):
            if childstate not in visited:    
                temppath=path.copy()
                temppath.append(action)
                frontier.push((childstate, temppath))
    return []


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    visited=[problem.getStartState()]
    frontier=util.Queue()
    frontier.push((problem.getStartState(),list()))
    while(not frontier.isEmpty()):
        newState, path = frontier.pop()
        if problem.isGoalState(newState):
            return path
        for childstate, action, cost in problem.getSuccessors(newState):
            if childstate not in visited:    
                temppath=path.copy()
                temppath.append(action)
                frontier.push((childstate, temppath))
                visited.append(childstate)
    return []

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    
    #Redefined the pqueue's update function using inheritance.
    #This redefinition is so it returns true or false if it actually
    #updated a value. (Opposed to just breaking and returning nothing)
    class ResponsivePQueue(util.PriorityQueue):
        def responsiveUpdate(self, item, priority):
            for index, (p, c, i) in enumerate(self.heap):
                if i == item:
                    if p <= priority:
                        return False
                    del self.heap[index]
                    self.heap.append((priority, c, item))
                    util.heapq.heapify(self.heap)
                    return True
            else:
                self.push(item, priority)
                return True

    #I used a graph that stores cheapest paths to two nodes.
    #This is for returning the path when getting to the goal.
    #Graph will hold costs for comparing which paths are cheaper.
    graph=dict()
    graph[problem.getStartState()]=(0,[])
    frontier=ResponsivePQueue()
    frontier.push(problem.getStartState(),0)
    while(not frontier.isEmpty()):
        newState = frontier.pop()
        if problem.isGoalState(newState):
            return graph[newState][1]
        for childstate, action, cost in problem.getSuccessors(newState):
            newcost=graph[newState][0]+cost
            if (childstate not in graph) or (graph[childstate][0] > newcost):
                newpath=graph[newState][1].copy()
                newpath.append(action)
                if frontier.responsiveUpdate(childstate, newcost):    
                    graph[childstate]=(newcost, newpath)

    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    #Redefined the pqueue's update function using inheritance.
    #This redefinition is so it returns true or false if it actually
    #updated a value. (Opposed to just breaking and returning nothing)
    class ResponsivePQueue(util.PriorityQueue):
        def responsiveUpdate(self, item, priority):
            for index, (p, c, i) in enumerate(self.heap):
                if i == item:
                    if p <= priority:
                        return False
                    del self.heap[index]
                    self.heap.append((priority, c, item))
                    util.heapq.heapify(self.heap)
                    return True
            else:
                self.push(item, priority)
                return True

    #I used a graph that stores cheapest paths to two nodes.
    #This is for returning the path when getting to the goal.
    #The graph also holds cost and estimates to recalculate the priority.
    graph=dict()
    graph[problem.getStartState()]=(0, heuristic(problem.getStartState(), problem), [])
    frontier=ResponsivePQueue()
    frontier.push(problem.getStartState(),0)
    while(not frontier.isEmpty()):
        newState = frontier.pop()
        if problem.isGoalState(newState):
            return graph[newState][2]
        for childstate, action, cost in problem.getSuccessors(newState):
            newcost=graph[newState][0]+cost
            newapprox=newcost+heuristic(childstate, problem)
            if (childstate not in graph) or (graph[childstate][1] > newapprox):
                newpath=graph[newState][2].copy()
                newpath.append(action)
                if frontier.responsiveUpdate(childstate, newapprox):    
                    graph[childstate]=(newcost, newapprox, newpath)

    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
