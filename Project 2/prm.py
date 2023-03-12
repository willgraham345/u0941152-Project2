#!/usr/bin/env python
'''
Package providing implementation of a probabilistic roadmap algorithm
'''

import numpy as np
import matplotlib.pyplot as plotter
from collisions import PolygonEnvironment
import time
import heapq
_DEBUG = False
_DEBUG_END = True

def fake_in_collision(q):
    '''
    We never collide with this function!
    '''
    return False

def euclidean_heuristic(self, s, goal):
    '''
    Euclidean heuristic function

    s - configuration vector
    goal - goal vector

    returns - floating point estimate of the cost to the goal from state s
    '''
    return np.linalg.norm(s - goal)

class PriorityQ:
    '''
    Priority queue implementation with quick access for membership testing
    Setup currently to only with the SearchNode class
    '''
    def __init__(self):
        '''
        Initialize an empty priority queue
        '''
        self.l = [] # list storing the priority q
        self.s = set() # set for fast membership testing

    def __contains__(self, x):
        '''
        Test if x is in the queue
        '''
        return x in self.s

    def push(self, x, cost):
        '''
        Adds an element to the priority queue.
        If the state already exists, we update the cost
        '''
        if tuple(x.state.tolist()) in self.s:
            return self.replace(x, cost)
        heapq.heappush(self.l, (cost, x))
        self.s.add(tuple(x.state.tolist()))

    def pop(self):
        '''
        Get the value and remove the lowest cost element from the queue
        '''
        x = heapq.heappop(self.l)
        self.s.remove(tuple(x[1].state.tolist()))
        return x[1]

    def peak(self):
        '''
        Get the value of the lowest cost element in the priority queue
        '''
        x = self.l[0]
        return x[1]

    def __len__(self):
        '''
        Return the number of elements in the queue
        '''
        return len(self.l)

    def replace(self, x, new_cost):
        '''
        Removes element x from the q and replaces it with x with the new_cost
        '''
        for y in self.l:
            if tuple(x.state.tolist()) == tuple(y[1].state.tolist()):
                self.l.remove(y)
                self.s.remove(tuple(y[1].state.tolist()))
                break
        heapq.heapify(self.l)
        self.push(x, new_cost)

    def get_cost(self, x):
        for y in self.l:
            if tuple(x.state.tolist()) == tuple(y[1].state.tolist()):
                return y[0]

    def __str__(self):
        return str(self.l)

def backpath(node):
    '''
    Function to determine the path that lead to the specified search node

    node - the SearchNode that is the end of the path

    returns - a tuple containing (path, action_path) which are lists respectively of the states
    visited from init to goal (inclusive) and the actions taken to make those transitions.
    '''
    path = []
    while node.parent is not None:
        path.append(node.state)
        node = node.parent
    path.append(node.state)
    path.reverse()
    return path

class StraightLinePlanner:
    def __init__(self, step_size, collision_func = None):
        self.in_collision = collision_func
        self.epsilon = step_size
        if collision_func is None:
            self.in_collision = fake_in_collision

    def plan(self, start, goal):
        '''
        Check if edge is collision free, taking epsilon steps towards the goal
        Returns: None / False if edge in collsion
                 Plan / True if edge if free
        '''
        # Check if goal or start is in collision
        if _DEBUG:
            print("start: ", start, " goal: ", goal, "")
        if self.in_collision(start) or self.in_collision(goal):
            return None, False
        
        length = np.linalg.norm(goal - start)
        if length < self.epsilon:
            return None, False
        else:
            for i in range(int(length / self.epsilon)):
                if self.in_collision(start + i * self.epsilon * (goal - start) / length):
                    return None, False
            return [start, goal], True
class RoadMapNode:
    '''
    Nodes to be used in a built RoadMap class
    '''
    def __init__(self, state, cost=0, parent=None):
        self.state = np.array(state)
        self.neighbors = []
        self.cost = 0
        self.parent = parent

    def add_neighbor(self, n_new):
        '''
        n_new - new neighbor
        '''
        self.neighbors.append(n_new)

    def is_neighbor(self, n_test):
        '''
        Test if n_test is already our neighbor
        '''
        for n in self.neighbors:
            if np.linalg.norm(n.state - n_test.state) == 0.0:
                return True
        return False

    def __eq__(self, other):
        return np.linalg.norm(self.state - other.state) == 0.0

class RoadMap:
    '''
    Class to store a built roadmap for searching in our multi-query PRM
    '''
    def __init__(self):
        self.nodes = []
        self.edges = []

    def add_node(self, node, neighbors):
        '''
        Add a node to the roadmap. Connect it to its neighbors
        '''
        # Avoid adding duplicates
        self.nodes.append(node)
        for n in neighbors:
            node.add_neighbor(n)
            if not n.is_neighbor(node):
                n.add_neighbor(node)
                self.edges.append((n.state, node.state))
                
    def get_states_and_edges(self):
        states = np.array([n.state for n in self.nodes])
        return (states, self.edges)

class PRM:
    def __init__(self, num_samples, local_planner, num_dimensions, lims = None,
                 collision_func = None, radius=2.0, epsilon=0.1):
        '''
        num_samples - number of samples to take
        local_planner - local planner to use for collision checking
        num_dimensions - number of dimensions of the state space
        lims - limits of the state space
        collision_func - function to test if a state is in collision
        radius - radius of the ball to use for k-nearest neighbors
        epsilon - step size for local planner
        '''
        self.local_planner = local_planner
        self.r = radius
        self.N = num_samples
        self.n = num_dimensions
        self.epsilon = epsilon
        self.start_node = None

        self.in_collision = collision_func
        if collision_func is None:
            self.in_collision = fake_in_collision

        # Setup range limits
        self.limits = lims
        if self.limits is None:
            self.limits = []
            for n in range(num_dimensions):
                self.limits.append([0,100])
            self.limits = np.array(self.limits)

        self.ranges = self.limits[:,1] - self.limits[:,0]

        # Build the roadmap instance
        self.T = RoadMap()
        self.search_node_list = []

    def build_prm(self, reset=False):
        '''
        reset - empty the current roadmap if requested
        '''
        if reset:
            self.T = RoadMap()
        i = 0
        while len(self.T.nodes) < self.N:
            i = i+1 # i = iteration
            q = self.sample()
            if not self.in_collision(q):
                valid_neighbors = self.find_valid_neighbors(q, self.T.nodes, self.r)
                self.T.add_node(RoadMapNode(q), valid_neighbors)
                # a = 0
                # for n in valid_neighbors:
                #     if n.is_neighbor(self.T.nodes[-1]):
                #         print("Already a neighbor dude")
                #         print("a = ", a)
                #         continue
                #     else:
                #         n.add_neighbor(self.T.nodes[-1])
                #         print('added new neighbors')
                #         a = a+1
                #         print("a = ", a, "new neighbors added!")
        for n in self.T.nodes:
            print(n.neighbors)
        return

    def find_valid_neighbors(self, n_query, samples, r):
        '''
        Find the nodes that are close to n_query and can be attached by the local planner
        returns - list of neighbors reached by the local planner
        n_query - query state
        samples - list of samples to check
        r - radius of the ball to search in
        goal - goal node
        goal.state - goal state
        '''
        valid_neighbors = []
        if _DEBUG:
            print('Finding valid neighbors for', n_query)
            print('Samples:', samples)
        
        for sample_nodes in samples: # iterates through through samples 
            if _DEBUG:
                print(sample_nodes.state)
            if np.linalg.norm(sample_nodes.state - n_query) < r: # if the distance between the sample and the query is less than the radius
                if _DEBUG:
                    print('less than radius')
                if self.local_planner.plan(n_query, sample_nodes.state): # if the local planner can plan a path between the sample and the query
                    valid_neighbors.append(sample_nodes)
                    if _DEBUG:
                        print('valid neighbor')
        if _DEBUG:
            print('Valid neighbors:', valid_neighbors)
        return valid_neighbors
    
    def query(self, start, goal):
        '''
        Generate a path from start to goal using the built roadmap
        returns - Path of configurations if in roadmap, None otherwise
        start - start state
        goal - goal state
        '''
        def is_goal(x):
            '''
            Test if a sample is at the goal
            '''
            return np.linalg.norm(x - goal) < self.epsilon
        self.start_node = self.init_start_and_goal_nodes(start, goal)
        
        # C
        # plan, visited = self.bfs(self.start_node, is_goal)
        # plan, visited = self.ucs(self.start_node, is_goal)
        plan, visited = self.a_star(self.start_node, is_goal)
        return plan, visited
    def init_start_and_goal_nodes(self, start, goal, r = None):
        # Create start and goal edges and nodes
        start = RoadMapNode(start)
        goal = RoadMapNode(goal)

        # Find valid neighbors for start and goal
        r = self.r
        while True:
            start_valid_neighbors = self.find_valid_neighbors(start.state, self.T.nodes, r)
            goal_valid_neighbors = self.find_valid_neighbors(goal.state, self.T.nodes, r)
            start_valid_neighbors = self.find_valid_neighbors(start.state, self.T.nodes, r)
            goal_valid_neighbors = self.find_valid_neighbors(goal.state, self.T.nodes, r)
            r = r*2
            if len(start_valid_neighbors) > 0 and len(goal_valid_neighbors) > 0:
                self.T.add_node(start, start_valid_neighbors)
                self.T.add_node(goal, goal_valid_neighbors)
                break
        print("Start and goal nodes initialized")
        return start
    def bfs(self, start, is_goal, K = 5000):
        '''
        Breadth first search, working :)
        '''
        # Initialize queue
        queue = []
        queue.append(start)
        visited = []
        i = 0
        while i < K:
            i = i + 1
            node = queue.pop(0)
            if node not in visited:
                visited.append(node)
                if is_goal(node.state):
                    print("Goal found")
                    return backpath(node), visited
                for neighbor in node.neighbors:
                    if neighbor not in visited:
                        neighbor.parent = node
                        queue.append(neighbor)
        print("No path found")
        return None, visited
    def uniform_cost_search(init_state, f, is_goal, actions):
        '''
        Perform uniform cost search on a grid map.

        init_state - the intial state on the map
        f - transition function of the form s_prime = f(s,a)
        is_goal - function taking as input a state s and returning True if its a goal state
        actions - set of actions which can be taken by the agent
        a_cost - the cost of taking action a (default is 1)

        returns - ((path, action_path), visited) or None if no path can be found
        path - a list of tuples. The first element is the initial state followed by all states
            traversed until the final goal state
        action_path - the actions taken to transition from the initial state to goal state
        '''
        frontier = PriorityQ()
        g_cost = 0 # g_cost is the cost to come to a node
        n0 = SearchNode(init_state, actions, None, None, g_cost)
        visited_set = {}
        frontier.push(n0, g_cost)
        while True:
            if frontier.__len__() == 0: # If frontier is empty, no path found
                print("No path found")
                print("Visited: ", visited_set)
                return ([[]], visited_set)
            
            # Get node lowest cost and its g_cost
            n_i = frontier.pop() # Gets the frontier node with the lowest cost
            path, action_path, g_cost = backpath(n_i)
            if _DEBUG:
                print("------------------------")
                print("Node of interest n_i: ", n_i.state, n_i.cost)
                if g_cost != n_i.cost: # If the g_cost is not the same as the node's cost, update the node's cost
                    raise ("g_cost is not the same as the node's cost")
            # adds to visited if not already visited
            if n_i.state not in visited_set:
                visited_set = update_visited_set(n_i, visited_set, g_cost)
                
                # If goal state, return path and visited set
                if is_goal(n_i.state): 
                    path_set = get_path_set(n_i, visited_set)
                    path, action_path, g_cost = backpath(n_i)
                    print("Path: ", path)
                    print("number of nodes in path: ", len(path))
                    print("Cost: ", g_cost)
                    return (path_set, visited_set)
                
                # If not goal state, add all children to frontier
                else: 
                    for a in actions:
                        if _DEBUG:
                            print("Action: ", a)
                            frontier.printStatesandCosts()
                            print("Visited: ", visited_set)
                        s_prime = f(n_i.state, a) # Get the state of the child node
                        frontier = handle_action(s_prime, actions, frontier, visited_set, n_i, a, g_cost)

    def a_star(self, start, is_goal, K = 5000):
        '''
        A* search
        '''
        # Initialize queue
        queue = []
        queue.append(start)
        visited = []
        i = 0
        while i < K:
            i = i + 1
            node = queue.pop(0)
            if node not in visited:
                visited.append(node)
                if is_goal(node.state):
                    print("Goal found")
                    return backpath(node), visited
                for neighbor in node.neighbors:
                    if neighbor not in visited:
                        neighbor.parent = node
                        queue.append(neighbor)
        print("No path found")
        return None, visited
    
    def sample(self):
        '''
        Sample a new configuration
        Returns a configuration of size self.n bounded in self.limits
        '''
        return np.random.rand(self.n) * self.ranges + self.limits[:,0]

def saveFig(name,close = True):
    plotter.savefig(name + ".png")
    if(close):
        plotter.close()

  
def test_prm_env(num_samples=50, step_length=1, env='./env0.txt'):
    pe = PolygonEnvironment()
    pe.read_env(env)

    dims = len(pe.start)
    start_time = time.time()

    local_planner = StraightLinePlanner(step_length, pe.test_collisions)
    prm = PRM(num_samples,
              local_planner,
              dims,
              radius = 20,
              epsilon = step_length,
              lims = pe.lims,
              collision_func=pe.test_collisions)
    print('Builing PRM')
    prm.build_prm()
    build_time = time.time() - start_time
    print('Build time', build_time)
    print('Finding Plan')
    plan, visited = prm.query(pe.start, pe.goal)
    pe.draw_env(show=False)
    pe.draw_plan(None, prm,False,True,True)

    run_time = time.time() - start_time
    print('plan:', plan)
    print('run_time =', run_time)


    return plan, prm, visited
if __name__== "__main__":
  test_prm_env()
