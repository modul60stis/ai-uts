#!/usr/bin/env python
# coding: utf-8

# In[1]:


# Import Library
import pandas as pd

# Import Data
data = pd.read_csv("https://raw.githubusercontent.com/melaniewalsh/sample-social-network-datasets/master/sample-datasets/modernist-journals-project/crisis/crisis-edges.csv", delimiter=',')


# In[2]:


# Menampilkan Data
data


# In[3]:


# Import Dictionary for Graph
from collections import defaultdict

# Function untuk menambahkan edge (sisi) pada Graph
graph = defaultdict(list)
def addEdge(graph, u, v):
  graph[u].append(v)

# Definition of Function
def generate_edges(graph):
  edges = []
  # for each node in graph (untuk setiap node dalam graph)
  for node in graph:
    # for each neighbour node of single node (untuk setiap node tetangga pada node tunggal)
    for neighbour in graph[node]:
      # if edge exists then append (jika edge ada maka tambahkan)
      edges.append((node, neighbour))
  return edges

# Declaration of graph as dictionary
for i in range(len(data)):
  addEdge(graph, data["Source"][i], data["Target"][i]) #menambahkan edge antara 2 node, yaitu Source dan Target


# In[4]:


# Print Generated Edges Graph
print(generate_edges(graph))


# In[5]:


graph.keys()


# In[6]:


# Print Generated Graph
graph


# In[ ]:





# In[7]:


# Membuat Fungsi DFS
def DFS(start, goal, path, level):
    print('\nLevel saat ini -->', level)
    print('Node tujuan untuk diuji', start)
    path.append(start)
    if start==goal:
        print('Goal test berhasil')
        return path
    print('Goal node test gagal')
    print('\nNode saat ini',start)
    for child in graph[start]:
        if DFS(child,goal,path,level+1):
            return path
        path.pop()
    return False

start = input('Start node: ') # Allison, M. G.
goal = input('Masukkan Tujuan : ') # Scurlock
print()
path = list()
res = DFS(start,goal,path,0)
if(res):
    print("Jalur ke goal telah ditemukan")
    print("path", path)
else:
    print("Tidak terdapat jalur yang ditemukan untuk batas kedalaman yang telah ditentukan")
    


# In[8]:


# Visualisasi Graph
#!pip install ipywidgets
#!pip install ipympl


# In[9]:


import networkx as nx
import matplotlib.pyplot as plt

get_ipython().run_line_magic('matplotlib', 'widget')

G = nx.Graph()

for i in range(len(data)):
  G.add_edge(data["Source"][i], data["Target"][i])


# In[10]:


print(nx.info(G))


# In[11]:


plt.figure()
nx.draw(G, with_labels = 1)

