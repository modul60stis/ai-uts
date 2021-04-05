#!/usr/bin/env python
# coding: utf-8

# ## Uniform Cost Search

# Uniform Cost Search adalah algoritma yang melakukan searching berdasarkan informasi cost aktual (yang sebenarnya). Next-state dipilih berdasarkan actual-cost terkecil atau terbesar (tergantung permasalahan). Pada akhirnya UCS akan menyisir semua nodes yang ada di dalam tree.

# # Code pertama

# ## Membuat Graf

# In[1]:


class Graph:

    def __init__(self, dict=None, directed=True):
        self.dict = dict or {}
        self.directed = directed
        if not directed:
            self.make_undirected()

    def make_undirected(self):
        """Make a digraph into an undirected graph by adding symmetric edges."""
        for a in list(self.dict.keys()):
            for (b, dist) in self.dict[a].items():
                self.connect1(b, a, dist)

    def connect(self, A, B, distance=1):
        """Add a link from A and B of given distance, and also add the inverse
        link if the graph is undirected."""
        self.connect1(A, B, distance)
        if not self.directed:
            self.connect1(B, A, distance)

    def connect1(self, A, B, distance):
        """Add a link from A to B of given distance, in one direction only."""
        self.dict.setdefault(A, {})[B] = distance

    def get(self, a, b=None):
        """Return a link distance or a dict of {node: distance} entries.
        .get(a,b) returns the distance or None;
        .get(a) returns a dict of {node: distance} entries, possibly {}."""
        links = self.dict.setdefault(a, {})
        if b is None:
            return links
        else:
            return links.get(b)

    def nodes(self):
        """Return a list of nodes in the graph."""
        return list(self.dict.keys())


def UndirectedGraph(dict=None):
    """Build a Graph where every edge (including future ones) goes both ways."""
    return Graph(dict=dict, directed=False)


# ### Membuat jarak antar kota dengan kota yang berdekatan

# Mendefinisikan jarak antar kota satu dengan kota berdekatan lainnya beserta koordinat lokasi pada 17 kota di Jerman.

# In[2]:


# membuat jarak antar kota dengan kota lainnya
germany_map = UndirectedGraph(dict(
    Luebeck=dict(Hamburg=63),
    Hamburg=dict(Bremen=116, Hannover=153, Berlin=291),
    Bremen=dict(Hannover=132, Dortmund=234),
    Hannover=dict(Magdeburg=148,Kassel=165),
    Magdeburg=dict(Berlin=166),
    Berlin=dict(Dresden=204),
    Dresden=dict(Leipzig=119),
    Leipzig=dict(Magdeburg=125,Nuremberg=263),
    Dortmund=dict(Duesseldorf=69,Saarbruecken=350),
    Kassel=dict(Frankfurt=185),
    Frankfurt=dict(Dortmund=221,Nuremberg=222),
    Saarbruecken=dict(Frankfurt=177,Karlsruhe=143),
    Karlsruhe=dict(Stuttgart=71),
    Stuttgart=dict(Frankfurt=200,Munich=215,Nuremberg=207),
    Nuremberg=dict(Munich=171)))

# membuat koordinat lokasi tiap kota
germany_map.locations = dict(
    Luebeck=(500,400),
    Hamburg=(455.04, 385.09),
    Bremen=(365.23,352.86),
    Dortmund=(335.12, 248.26),
    Duesseldorf=(287.97,237.4),
    Saarbruecken=(279.16,119.06),
    Frankfurt=(425.5,167.76),
    Hannover=(449.45,314.78),
    Kassel=(475.97,231.43),
    Karlsruhe=(384.57,97.76),
    Stuttgart=(457.86,91.35),
    Munich=(582.32,88.47),
    Nuremberg=(565.09,154.85),
    Leipzig=(580.24,246.76),
    Magdeburg=(558.16,296.43),
    Dresden=(666.97,242.81),
    Berlin=(639.37,346.89))


# ### Mendefinisikan Problem

# Problem : mencari rute terpendek dari kota Luebeck ke kota Munich. Berikut fungsi mendefinisikan problem dan graf nya :

# In[3]:


class Problem(object):

    def __init__(self, initial, goal=None):
        self.initial = initial
        self.goal = goal

    def actions(self, state):
        raise NotImplementedError

    def result(self, state, action):
        raise NotImplementedError

    def goal_test(self, state):
        if isinstance(self.goal, list):
            return is_in(state, self.goal)
        else:
            return state == self.goal

    def path_cost(self, c, state1, action, state2):
        return c + 1

    def value(self, state):
        raise NotImplementedError


# In[4]:


class GraphProblem(Problem):

    def __init__(self, initial, goal, graph):
        Problem.__init__(self, initial, goal)
        self.graph = graph

    def actions(self, A):
        """The actions at a graph node are just its neighbors."""
        return list(self.graph.get(A).keys())

    def result(self, state, action):
        """The result of going to a neighbor is just that neighbor."""
        return action

    def path_cost(self, cost_so_far, A, action, B):
        return cost_so_far + (self.graph.get(A, B) or infinity)

    def find_min_edge(self):
        """Find minimum value of edges."""
        m = infinity
        for d in self.graph.dict.values():
            local_min = min(d.values())
            m = min(m, local_min)

        return m

    def h(self, node):
        """h function is straight-line distance from a node's state to goal."""
        locs = getattr(self.graph, 'locations', None)
        if locs:
            if type(node) is str:
                return int(distance(locs[node], locs[self.goal]))

            return int(distance(locs[node.state], locs[self.goal]))
        else:
            return infinity


# Dengan fungsi tersebut, definisikan germany problem beserta koordinatnya :

# In[5]:


germany_problem = GraphProblem('Luebeck', 'Munich', germany_map)


# In[6]:


germany_locations = germany_map.locations
print(germany_locations)


# ### Visualisasi graf Jerman

# Selanjutnya, melihat bagaimana frontier (m) meluas di setiap algoritme penelusuran untuk masalah sederhana bernama `germany_problem`. Mengimport modul yang dibutuhkan terlebih dahulu.

# In[7]:


get_ipython().run_line_magic('matplotlib', 'inline')
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib import lines

from ipywidgets import interact
import ipywidgets as widgets
from IPython.display import display
import time


# Dimulai dengan graf kosong, lalu tambahkan nodes (simpul pada graf). Tempatkan nodes tersebut sesuai posisi lokasi pada peta kota Jerman, kemudian tambahkan edge (garis pada graf).

# In[8]:


# initialise a graph
G = nx.Graph()

# use this while labeling nodes in the map
node_labels = dict()
# use this to modify colors of nodes while exploring the graph.
# This is the only dict we send to `show_map(node_colors)` while drawing the map
node_colors = dict()

for n, p in germany_locations.items():
    # add nodes from germany_locations
    G.add_node(n)
    # add nodes to node_labels
    node_labels[n] = n
    # node_colors to color nodes while exploring germany map
    node_colors[n] = "black"

# we'll save the initial node colors to a dict to use later
initial_node_colors = dict(node_colors)
    
# positions for node labels
node_label_pos = { k:[v[0],v[1]-10]  for k,v in germany_locations.items() }

# use this while labeling edges
edge_labels = dict()

# add edges between cities in germany map - UndirectedGraph defined in search.py
for node in germany_map.nodes():
    connections = germany_map.get(node)
    for connection in connections.keys():
        distance = connections[connection]

        # add edges to the graph
        G.add_edge(node, connection)
        # add distances to edge_labels
        edge_labels[(node, connection)] = distance


# Kita telah menyelesaikan pembuatan graf yang sesuai dengan peta di kota jerman. Fungsi yang digunakan adalah `show_map( )` dengan argumen node colors. Kemudian panggil fungsi ini untuk menampilkan peta pada setiap langkah interval saat proses searching, menggunakan algoritma UCS.

# In[9]:


def show_map(node_colors):
    # set the size of the plot
    plt.figure(figsize=(18,13))
    # draw the graph (both nodes and edges) with locations from germany_locations
    nx.draw(G, pos = germany_locations, node_color = [node_colors[node] for node in G.nodes()])

    # draw labels for nodes
    node_label_handles = nx.draw_networkx_labels(G, pos = node_label_pos, labels = node_labels, font_size = 14)
    # add a white bounding box behind the node labels
    [label.set_bbox(dict(facecolor='white', edgecolor='none')) for label in node_label_handles.values()]

    # add edge lables to the graph
    nx.draw_networkx_edge_labels(G, pos = germany_locations, edge_labels=edge_labels, font_size = 14)
    
    # add a legend
    black_circle = lines.Line2D([], [], color="black", marker='o', markersize=15, markerfacecolor="black")
    orange_circle = lines.Line2D([], [], color="orange", marker='o', markersize=15, markerfacecolor="orange")
    red_circle = lines.Line2D([], [], color="red", marker='o', markersize=15, markerfacecolor="red")
    blue_circle = lines.Line2D([], [], color="blue", marker='o', markersize=15, markerfacecolor="blue")
    green_circle = lines.Line2D([], [], color="green", marker='o', markersize=15, markerfacecolor="green")
    plt.legend((black_circle, orange_circle, red_circle, blue_circle, green_circle),
               ('Node yang belum dikunjungi', 'Node selanjutnya yang mungkin', 'Node yang sedang dikunjungi', 'Node yang sudah dikunjungi', 'Hasil'),
               numpoints=1,prop={'size':16}, loc=(1.0,.75))
    
    # show the plot. No need to use in notebooks. nx.draw will show the graph itself.
    plt.show()


# Setelah itu, kita dapat menampilkan objek (berupa graf) yang dibuat dengan memanggil fungsi `show_map( )`. Argumen dalam `show_map( )` merupakan sebuah variabel dictionary dengan key dan value berupa warna.

# In[10]:


show_map(node_colors)


# ### Visualisasi interaktif

# Pada tahap ini kita akan melakukan penerapan algoritma pencarian untuk menyelesaikan masalah kita menggunakan algoritma UCS (Uniform Cost Search). Setelah itu kita juga akan melakukan visualisasi bagaimana algoritma tersebut bekerja dalam menyelesaikan masalah kita.
# 
# Untuk menambahkan estetika pada visualisasi yang kita buat, kita akan menambahkan warna. Berikut ada warna untuk visualisasi dari proses searching uniform cost search beserta keterangannya.
# 
# - Node yang belum dikunjungi (hitam)
# - Node selanjutnya yang mungkin (orange)
# - Node yang Sedang dikunjungi (merah)
# - Node yang Sudah dikunjungi (biru)
# - Hasil (hijau)
# 
# Selanjutnya kita akan menentukan methode bantuan untuk menampilkan tombol dan slider interaktif dalam visualisasi kita.

# In[11]:


def final_path_colors(problem, solution):
    "returns a node_colors dict of the final path provided the problem and solution"
    
    # get initial node colors
    final_colors = dict(initial_node_colors)
    # color all the nodes in solution and starting node to green
    final_colors[problem.initial] = "green"
    for node in solution:
        final_colors[node] = "green"  
    return final_colors


def display_visual(user_input, algorithm=None, problem=None):
    if user_input == False:
        def slider_callback(iteration):
            # don't show graph for the first time running the cell calling this function
            try:
                show_map(all_node_colors[iteration])
            except:
                pass
        def visualize_callback(Visualize):
            if Visualize is True:
                button.value = False
                
                global all_node_colors
                
                iterations, all_node_colors, node = algorithm(problem)
                solution = node.solution()
                all_node_colors.append(final_path_colors(problem, solution))
                
                slider.max = len(all_node_colors) - 1
                
                for i in range(slider.max + 1):
                    slider.value = i
                     #time.sleep(.5)
        
        slider = widgets.IntSlider(min=0, max=1, step=1, value=0)
        slider_visual = widgets.interactive(slider_callback, iteration = slider)
        display(slider_visual)

        button = widgets.ToggleButton(value = False)
        button_visual = widgets.interactive(visualize_callback, Visualize = button)
        display(button_visual)
    
    if user_input == True:
        node_colors = dict(initial_node_colors)
        if algorithm == None:
            algorithms = {"Breadth First Tree Search": breadth_first_tree_search,
                          "Depth First Tree Search": depth_first_tree_search,
                          "Breadth First Search": breadth_first_search,
                          "Depth First Graph Search": depth_first_graph_search,
                          "Uniform Cost Search": uniform_cost_search,
                          "A-star Search": astar_search}
            algo_dropdown = widgets.Dropdown(description = "Search algorithm: ",
                                             options = sorted(list(algorithms.keys())),
                                             value = "Breadth First Tree Search")
            display(algo_dropdown)
        
        def slider_callback(iteration):
            # don't show graph for the first time running the cell calling this function
            try:
                show_map(all_node_colors[iteration])
            except:
                pass
            
        def visualize_callback(Visualize):
            if Visualize is True:
                button.value = False
                
                problem = GraphProblem(start_dropdown.value, end_dropdown.value, romania_map)
                global all_node_colors
                
                if algorithm == None:
                    user_algorithm = algorithms[algo_dropdown.value]
                
#                 print(user_algorithm)
#                 print(problem)
                
                iterations, all_node_colors, node = user_algorithm(problem)
                solution = node.solution()
                all_node_colors.append(final_path_colors(problem, solution))

                slider.max = len(all_node_colors) - 1
                
                for i in range(slider.max + 1):
                    slider.value = i
#                     time.sleep(.5)
                         
        start_dropdown = widgets.Dropdown(description = "Start city: ",
                                          options = sorted(list(node_colors.keys())), value = "Arad")
        display(start_dropdown)

        end_dropdown = widgets.Dropdown(description = "Goal city: ",
                                        options = sorted(list(node_colors.keys())), value = "Fagaras")
        display(end_dropdown)
        
        button = widgets.ToggleButton(value = False)
        button_visual = widgets.interactive(visualize_callback, Visualize = button)
        display(button_visual)
        
        slider = widgets.IntSlider(min=0, max=1, step=1, value=0)
        slider_visual = widgets.interactive(slider_callback, iteration = slider)
        display(slider_visual)


# ## Implementasi Uniform Cost Search

# Import modul dan definisikan fungsi yang dibutuhkan untuk menampilkan visualisasi interaktif pada graf Jerman.

# In[12]:


import bisect
def memoize(fn, slot=None, maxsize=32):
    """Memoize fn: make it remember the computed value for any argument list.
    If slot is specified, store result in that slot of first argument.
    If slot is false, use lru_cache for caching the values."""
    if slot:
        def memoized_fn(obj, *args):
            if hasattr(obj, slot):
                return getattr(obj, slot)
            else:
                val = fn(obj, *args)
                setattr(obj, slot, val)
                return val
    else:
        @functools.lru_cache(maxsize=maxsize)
        def memoized_fn(*args):
            return fn(*args)

    return memoized_fn

class Node:

    def __init__(self, state, parent=None, action=None, path_cost=0):
        """Create a search tree Node, derived from a parent by an action."""
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node {}>".format(self.state)

    def __lt__(self, node):
        return self.state < node.state

    def expand(self, problem):
        """List the nodes reachable in one step from this node."""
        return [self.child_node(problem, action)
                for action in problem.actions(self.state)]

    def child_node(self, problem, action):
        """[Figure 3.10]"""
        next = problem.result(self.state, action)
        return Node(next, self, action,
                    problem.path_cost(self.path_cost, self.state,
                                      action, next))

    def solution(self):
        """Return the sequence of actions to go from the root to this node."""
        return [node.action for node in self.path()[1:]]

    def path(self):
        """Return a list of nodes forming the path from the root to this node."""
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    # We want for a queue of nodes in breadth_first_search or
    # astar_search to have no duplicated states, so we treat nodes
    # with the same state as equal. [Problem: this may not be what you
    # want in other contexts.]

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        return hash(self.state)

class Queue:

    def __init__(self):
        raise NotImplementedError

    def extend(self, items):
        for item in items:
            self.append(item)

    
class PriorityQueue(Queue):

    def __init__(self, order=min, f=lambda x: x):
        self.A = []
        self.order = order
        self.f = f

    def append(self, item):
        bisect.insort(self.A, (self.f(item), item))

    def __len__(self):
        return len(self.A)

    def pop(self):
        if self.order == min:
            return self.A.pop(0)[1]
        else:
            return self.A.pop()[1]

    def __contains__(self, item):
        return any(item == pair[1] for pair in self.A)

    def __getitem__(self, key):
        for _, item in self.A:
            if item == key:
                return item

    def __delitem__(self, key):
        for i, (value, item) in enumerate(self.A):
            if item == key:
                self.A.pop(i)

def best_first_graph_search(problem, f):
    
    # we use these two variables at the time of visualisations
    iterations = 0
    all_node_colors = []
    node_colors = dict(initial_node_colors)
    
    f = memoize(f, 'f')
    node = Node(problem.initial)
    
    node_colors[node.state] = "red"
    iterations += 1
    all_node_colors.append(dict(node_colors))
    
    if problem.goal_test(node.state):
        node_colors[node.state] = "green"
        iterations += 1
        all_node_colors.append(dict(node_colors))
        return(iterations, all_node_colors, node)
    
    frontier = PriorityQueue(min, f)
    frontier.append(node)
    
    node_colors[node.state] = "orange"
    iterations += 1
    all_node_colors.append(dict(node_colors))
    
    explored = set()
    while frontier:
        node = frontier.pop()
        
        node_colors[node.state] = "red"
        iterations += 1
        all_node_colors.append(dict(node_colors))
        
        if problem.goal_test(node.state):
            node_colors[node.state] = "green"
            iterations += 1
            all_node_colors.append(dict(node_colors))
            return(iterations, all_node_colors, node)
        
        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
                node_colors[child.state] = "orange"
                iterations += 1
                all_node_colors.append(dict(node_colors))
            elif child in frontier:
                incumbent = frontier[child]
                if f(child) < f(incumbent):
                    del frontier[incumbent]
                    frontier.append(child)
                    node_colors[child.state] = "orange"
                    iterations += 1
                    all_node_colors.append(dict(node_colors))

        node_colors[node.state] = "blue"
        iterations += 1
        all_node_colors.append(dict(node_colors))
    return None

def uniform_cost_search(problem):
    "[Figure 3.14]"
    iterations, all_node_colors, node = best_first_graph_search(problem, lambda node: node.path_cost)
    return(iterations, all_node_colors, node)


# Berikut visualisasi node yang dilalui dari kota Luebeck ke kota Munich menggunakan UCS :

# In[13]:


all_node_colors = []
germany_problem = GraphProblem('Luebeck', 'Munich', germany_map)
display_visual(user_input = False, algorithm = uniform_cost_search, problem = germany_problem)


# Contoh lain mencari rute terpendek dari Luebeck ke Frankfurt :

# In[14]:


all_node_colors = []
germany_problem = GraphProblem('Luebeck', 'Frankfurt', germany_map)
display_visual(user_input = False, algorithm = uniform_cost_search, problem = germany_problem)


# #### ===================================================================================================================

# # Code kedua:

# ## **Uniform Cost Search**
# 
# Sumber : https://cyluun.github.io/blog/uninformed-search-algorithms-in-python

# **Import Data**

# In[15]:


import pandas as pd


# In[16]:


data = pd.read_csv("https://raw.githubusercontent.com/arinams/Artificial-Intelligence/main/Romania%20Map.csv")
data.head()


# **Mendefinisikan Graph**

# In[17]:


class Graph:
    def __init__(self):
        self.edges = {}
        self.weights = {}

    def neighbors(self, node):
        return self.edges[node]

    def get_cost(self, from_node, to_node):
        return self.weights[(from_node, to_node)]


# **Merubah Data Menjadi Graph**
# 
# Sumber : https://www.geeksforgeeks.org/generate-graph-using-dictionary-python/

# In[18]:


from collections import defaultdict

edges = defaultdict(list)
weight = defaultdict(int)

def addEdge(edges,u,v):
    edges[u].append(v)

def addWeight(weight, u, v, w):
    weight[u, v] = w

for i in range(len(data)):
    addEdge(edges,data["Source"][i], data["Target"][i])
    addWeight(weight, data["Source"][i], data["Target"][i], data["Weight"][i])

graph = Graph()
graph.edges = edges
graph.weights = weight


# **Fungsi Uniform Cost Search**
# 
# 1. Masukkan node awal ke dalam queue
# 2. Ulangi selama queue tidak kosong:
#   -	Hapus elemen berikutnya dengan prioritas tertinggi dari queue
#   -	Jika node adalah node tujuan, maka cetak cost dan path dan kemudian exit
#   -	Selain itu, masukkan semua turunan dari elemen yang dihapus ke dalam antrian dengan biaya kumulatifnya sebagai prioritas mereka.
# 

# In[19]:


from queue import PriorityQueue


# In[20]:


def ucs(graph, start, goal):
    visited = set()
    queue = PriorityQueue()
    queue.put((0, start, [start]))

    while queue:
        cost, node, path = queue.get()
        if node not in visited:
            print("Checking node", node)
            visited.add(node)

            if node == goal:
                print("Destination node found!")
                print("Path :", path)
                print("Total Cost :", cost)
                return
            for i in graph.neighbors(node):
                if i not in visited:
                    total_cost = cost + graph.get_cost(node, i)
                    queue.put((total_cost, i, path + [i]))


# In[21]:


ucs(graph, 'Arad', 'Bucharest')


# In[ ]:




