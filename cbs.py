import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    # Need to keep track of the shorter path as it will stay at its goal location until the other one finishes.
    # path1 will be longer than path2
    tmp_path1 = path1.copy()
    tmp_path2 = path2.copy()
    if len(tmp_path1) >= len(tmp_path2): # Compare length of 2 paths to control and update the goal location of the shorter path
        for i in range(len(tmp_path1)):
            if i >= len(tmp_path2):   # Updating goal location: append goal location to shorter path until iterate through all path1
                tmp_path2.append(tmp_path2[i-1])
            # print("path1: ", path1[i]," --- path2: ", path2[i] )
            # Two agents are at the same cell at the same time
            if tmp_path1[i] == tmp_path2[i]:
                # print("path1: ", path1[i]," --- path2: ", path2[i] )
                return [tmp_path1[i]], i
            #Two agents swap locations
            if i > 0 and tmp_path1[i-1] == tmp_path2[i] and tmp_path1[i] == tmp_path2[i-1]:
                return [tmp_path1[i-1], tmp_path1[i]], i
    else:  #path1 is shorter than path2
        for i in range(len(tmp_path2)):
            if i >= len(tmp_path1):
                tmp_path1.append(tmp_path1[i-1])
            # Two agents are at the same cell at the same time
            if tmp_path1[i] == tmp_path2[i]:
                return [tmp_path1[i]], i
            #Two agents swap locations
            if tmp_path1[i-1] == tmp_path2[i] and tmp_path1[i] == tmp_path2[i-1]:
                return [tmp_path1[i-1], tmp_path1[i]], i
    return None,None  #return -1,-1 if they do not have collision


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collision_lists = []
    for i in range(len(paths)-1):
        for j in range(i+1, len(paths)):
            loc, timestep = detect_collision(paths[i], paths[j])
            # Check if they have collision or not    
            if (loc is not None) and (timestep is not None):
                collision_lists.append({'a1': i, 'a2': j, 'loc': loc, 'timestep': timestep})
    return collision_lists


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    constraint = []
    if len(collision['loc']) == 1:
        constraint =[{'agent': collision['a1'],
                      'loc': collision['loc'],
                      'timestep': collision['timestep']},
                      {'agent': collision['a2'],
                       'loc': collision['loc'],
                       'timestep': collision['timestep']
                    }]
    else: #it's an edge constraint
        reversed_loc = collision['loc'][::-1] 
        constraint =[{'agent': collision['a1'],
                      'loc': collision['loc'],
                      'timestep': collision['timestep']},
                     {'agent': collision['a2'],
                      'loc': reversed_loc,  #reverse location for agent 2
                      'timestep': collision['timestep']
                    }]
    return constraint


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [{"earliest_goal_timestep": -1, "algo": "CBS"}],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')

            root['paths'].append(path)
            # if  root['constraints'][0]['earliest_goal_timestep'] == -1 or root['constraints'][0]['earliest_goal_timestep'] > len(path):
            #     root['constraints'][0]['earliest_goal_timestep'] = len(path) -1

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)


        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))
        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        while len(self.open_list) > 0:
            curr = self.pop_node()
            print("\n ----CURRENT NODE----: ", curr)
            
            #Return if the node has no collision
            if len(curr['collisions']) == 0:
                self.print_results(curr)
                return curr['paths']

            collision = curr['collisions'][0]
            constraints = standard_splitting(collision)
            for constraint in constraints:
                child = {'cost': 0,
                        'constraints': [],
                        'paths': [],
                        'collisions': []}

                child['constraints'] = curr['constraints'].copy()
                child['constraints'].append(constraint)
                child['paths'] = curr['paths'].copy()
                agent = constraint['agent']
                print("AGENT: ", agent)
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                            agent, child['constraints'])

                if path is not None:
                    #Replace path of agent ai in current path
                    child['paths'][agent] = path
                    #Update earliest_goal_timestep
                    # if child['constraints'][0]['earliest_goal_timestep'] == -1 or child['constraints'][0]['earliest_goal_timestep'] > len(path):    
                    #         child['constraints'][0]['earliest_goal_timestep'] = len(path) -1                                                    
                    child['collisions'] = detect_collisions(child['paths'])                                                                     
                    child['cost'] = get_sum_of_cost(child['paths'])
                    print("child node is: ", child)
                    self.push_node(child)
        return 'No Solution'
        # self.print_results(root)
        # return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
        