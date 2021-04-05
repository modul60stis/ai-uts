#!/usr/bin/env python
# coding: utf-8

# In[1]:


class Node:
    def __init__(self, name, heuristic=0, is_goal=False):
        self.name = name
        self.children = {}  # this will be a list of nodes with their corresponding distance
        self.heuristic = heuristic  # this will be used in greedy best first
        self.visited = False
        self.parent = None  # this will be used to find the path
        self.costFromOrigin = 0


# In[2]:


class RoutingBetweenCities:
    def __init__(self):
        # holds each city with it's neighbours and their distance
        self.initialStateAdjacencyList = {"Padangsidempuan": {"Sibolga": 86, "Gn.Tua":59, "Sipirok": 37},
                                          "Sibolga": {"Tarutung": 66, "Padangsidempuan": 86},
                                          "Gn.Tua": {"Kota Pinang": 89, "Padangsidempuan": 59, "Sipirok": 66},
                                          "Sipirok": {"Tarutung": 74, "Padangsidempuan": 37},
                                          "Tarutung": {"Siborong-borong": 25, "Sibolga": 66, "Sipirok": 74},
                                          "Kota Pinang": {"Rantau Prapat": 55, "Gn.Tua": 89},
                                          "Siborong-borong": {"Balige": 23 , "Tarutung": 25},
                                          "Rantau Prapat": {"Kisaran": 132, "Kota Pinang": 55},
                                          "Balige": {"Parapat": 60, "Siborong-borong": 23},
                                          "Kisaran": {"Lima Puluh": 39, "Rantau Prapat": 132},
                                          "Parapat": {"P.Siantar": 48, "Balige": 60},
                                          "Lima Puluh": {"Tebing Tinggi": 44, "P.Siantar": 52, "Kisaran": 39},
                                          "P.Siantar": {"Tebing Tinggi": 63, "Lima Puluh": 52, "Parapat": 48},
                                          "Tebing Tinggi": {"Lubuk Pakam": 50, "Lima Puluh": 44, "P.Siantar": 63},
                                          "Lubuk Pakam": {"Medan": 30, "Tebing Tinggi": 50},
                                          "Medan": {"Binjai": 22, "Lubuk Pakam": 30},
                                          "Binjai": {"Medan": 22}}

        # heuristic distance of each city to goal(Medan)
        self.initialStateHeuristics = {"Padangsidempuan": 367, "Sibolga": 325, "Gn.Tua": 435, "Sipirok": 327, 
                                       "Tarutung": 253, "Kota Pinang": 336, "Siborong-borong": 244, 
                                       "Rantau Prapat": 283, "Balige": 212, "Kisaran": 165, "Parapat": 242, 
                                       "Lima Puluh": 127, "P.Siantar": 132, "Tebing Tinggi": 80, "Lubuk Pakam": 36, 
                                       "Medan": 0, "Binjai": 21}

    def get_initial_graph(self):
        # creating a dictionary based on city names
        # each entry of this dict holds a node that's created with this name [i've use it kinda like hashMap in java]
        graph_details_dictionary = {}
        # creating graph nodes with their names and heuristic values
        for x in self.initialStateAdjacencyList:
            current_node = Node(name=x, heuristic=self.initialStateHeuristics[x])
            graph_details_dictionary.update({x: current_node})

        # fill children field of each node and assign pointers to it's children with it's corresponding cost
        # children field is a dictionary consist of children nodes and their cost
        # in this form -> a.children = {"city a" : 200 , "city b" : 400}
        for x in graph_details_dictionary:
            for y in self.initialStateAdjacencyList[x]:
                graph_details_dictionary[x].children.update(
                    {graph_details_dictionary[y]: self.initialStateAdjacencyList[x][y]})  # update children dictionary
                # like the form specified above :{"child city x" :200}

        return graph_details_dictionary


if __name__ == '__main__':
    routingBetweenCities = RoutingBetweenCities()
    a = routingBetweenCities.get_initial_graph()
    print()


# In[3]:


def path_to_destination(destination_node):
    # Iterative Implementation ------------>
    if destination_node.parent is None:  # it may happen in LDFS with not sufficient depth
        print("There's no path with the given situation to destination")
        return
    node = destination_node
    path_to_dest = []
    while node is not None:
        path_to_dest.append(node)
        node = node.parent
    path_to_dest.reverse()
    for x in path_to_dest:
        print(x.name, "=>", end=" ")
    print()
    return path_to_dest


# ### Greedy Best First Search

# In[4]:


import queue as queue

def greedy_best_first_search(start_node, dest_node):
    expanded_nodes = []  # to compare it with other algorithms
    visited_nodes_queue = queue.PriorityQueue()  # to sort them base on their heuristic
    maximum_memory_usage = 0  # to compare it with other algorithms
    expanded_nodes.append(start_node)
    start_node.visited = True
    current_node = start_node
    while True:
        # check if we reach to dest or not
        if dest_node.visited:
            break
        for x in current_node.children:
            if not x.visited:
                x.visited = True
                x.parent = current_node
                # x's cost from origin = parent's costFromOrigin from origin + distance between'em
                x.costFromOrigin = current_node.costFromOrigin + current_node.children[x]
                visited_nodes_queue.put((x.heuristic, x))
        maximum_memory_usage = max(maximum_memory_usage, visited_nodes_queue.qsize())
        current_node = visited_nodes_queue.get(0)[1]
        expanded_nodes.append(current_node)
    return expanded_nodes.__len__(), visited_nodes_queue.qsize(), maximum_memory_usage


# In[5]:


def main():
    problem = RoutingBetweenCities()
    graph_city_details_dictionary = problem.get_initial_graph()
    # start_node = graph_city_details_dictionary["Padangsidempuan"]
    dest_node = graph_city_details_dictionary["Medan"]
    # expanded_nodes = 0
    # visited_nodes = 0
    # max_memory_usage = 0
    # define mode to perform algorithm
    mode = "Graph"
    algorithm = "GREEDY BEST FIRST SEARCH"
    expanded_nodes, visited_nodes, max_memory_usage = algorithms_switcher(graph_mode_or_tree_mode=mode,
                                                                          algorithm=algorithm
                                                                          ,
                                                                          graph_city_details_dictionary=
                                                                          graph_city_details_dictionary
                                                                          )

    # trace back path from destination
    path_to_dest = path_to_destination(destination_node=dest_node)

    if dest_node.costFromOrigin != 0:
        cost = dest_node.costFromOrigin  # for cost based algorithms :a*,uniform cost ,...
    else:
        cost = path_to_dest.__len__() - 1  # for non cost based algorithm :dfs,bfs
    print("observed nodes : ", visited_nodes)
    print("expanded nodes : ", expanded_nodes)
    print("maximum memory usage based on nodes : ", max_memory_usage)
    print("path cost : ", cost)


def algorithms_switcher(graph_mode_or_tree_mode, algorithm, graph_city_details_dictionary):
    start_node = graph_city_details_dictionary["Padangsidempuan"]
    dest_node = graph_city_details_dictionary["Medan"]
    expanded_nodes = 0
    visited_nodes = 0
    max_memory_usage = 0
    if graph_mode_or_tree_mode == "Graph":
        # Graph
        print("traversal of graph is : ")
        if algorithm == "GREEDY BEST FIRST SEARCH":
            expanded_nodes, unexpanded_nodes, max_memory_usage = greedy_best_first_search(start_node=start_node,
                                                                                          dest_node=dest_node)
            visited_nodes = expanded_nodes + unexpanded_nodes

    return expanded_nodes, visited_nodes, max_memory_usage


if __name__ == '__main__':
    main()

