#!/usr/bin/env python
# coding: utf-8

# # Implementasi Depth Limited Search
# 

# ## Memuat Data
# 
# Network data for relationships between President Donald Trump and other people, which was originally compiled by John Templon, Anthony Cormier, Alex Campbell, and Jeremy Singer-Vine as part of a larger project of mapping "TrumpWorld" for BuzzFeed News.
# 
# The data was compiled by culling from "public records, news reports, and other sources on the Trump family, his Cabinet picks, and top advisers," as well as via crowdsourced tips and information from the public.
# 
# As Templon, Cormier, Campbell, and Singer-Vine suggest, "No American president has taken office with a giant network of businesses, investments, and corporate connections like that amassed by Donald J. Trump."

# In[1]:


#import library
import pandas as pd

#load data
df = pd.read_csv("https://raw.githubusercontent.com/melaniewalsh/sample-social-network-datasets/master/sample-datasets/trump/trump-edges.csv", delimiter=',')
df


# ## Membentuk Graph
# 
# Mengaplikasikan *graph* menggunakan struktur data `dictionary`. `Keys` digunakan sebagai `nodes` dari *graph* dan `values`-nya adalah `lists` tiap `node` yang terhubung menggunakan `edge`. Contoh:
# 
# ```
# graph = { "a" : ["c"],
#           "b" : ["c", "e"],
#           "c" : ["a", "b", "d", "e"],
#           "d" : ["c"],
#           "e" : ["c", "b"],
#           "f" : []
#         }
# ```

# In[2]:


#import dictionary for graph
from collections import defaultdict
  
#function for adding edge to graph 
graph = defaultdict(list)
def addEdge(graph,u,v): 
    graph[u].append(v)
  
#declaration of graph as dictionary 
for i in range(len(df)):
  addEdge(graph,df["Source"][i],df["Target"][i])


# In[3]:


#print generated graph
graph


# ## Visualisasi Graph
# 
# Memvisualisasikan *graph* menggunakan bantuan *library* `networkx` dan `matplotlib`.

# In[4]:


#!pip install ipywidgets
#!pip install ipympl


# In[5]:


#import library
import networkx as nx
import matplotlib.pyplot as plt

#interactive plot
get_ipython().run_line_magic('matplotlib', 'notebook')

#create graph
G = nx.Graph()
for i in range(len(df)):
  G.add_edge(df["Source"][i],df["Target"][i])


# In[6]:


#print info
print(nx.info(G))


# In[7]:


#draw graph
plt.figure()
nx.draw(G, with_labels=1, edge_color='red')


# ## Membuat Fungsi DLS

# In[8]:


#function for depth limited search
def DLS(start,goal,path,level,maxD):
  print('\nLevel: ',level)
  print('Goal node testing for',start)
  path.append(start)
  #path = ['Donald J. Trump']
  
  if start == goal:
    print("Goal test successful")
    return path
  print('Goal node testing failed')

  #misal level = 3, maxD = 3
  #pindah cabang
  if level==maxD:
    return False
  print('\nExpanding the current node',start)
  for child in graph[start]:
    if DLS(child,goal,path,level+1,maxD):
      return path
    path.pop()
  return False


# ## Menerapkan DLS

# In[9]:


start = "Donald J. Trump"
goal = input('Enter the goal node: ') #Angela Chao
path = list()
level = 0
maxD = int(input("Enter the maximum depth limit: "))

res = DLS(start,goal,path,level,maxD)
if(res):
    print("Path to goal node available")
    print("Path",path)
else:
    print("No path available for the goal node in given depth limit")


# ## Visualisasi Path

# In[10]:


H = nx.Graph()

for i in range(1,len(path)):
  H.add_edge(path[i-1],path[i])


# In[11]:


print(nx.info(H))


# In[12]:


plt.figure()
nx.draw(H, with_labels=1, edge_color='red')

