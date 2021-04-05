#!/usr/bin/env python
# coding: utf-8

# ## 8-puzzle menggunakan Greedy Best First Search

# In[82]:


import copy
import time


# In[111]:


# menentukan initial state dan goal state
startNode = [1,4,2, 
             3,7,5, 
             6,0,8]
finalNode = [0,1,2, 
             3,4,5, 
             6,7,8]


# Berikut ini adalah memeriksa apakah initial state memiliki solusi berupa goal state atau tidak. Pengujian ini akan menggunakan permutasi paritas

# In[113]:


def hasSolution(init, goal):
  total_init = 0
  total_goal = 0
  for i in range(8):
    for j in range(i+1, 9):

      if init[i] > 0 and init[j] > 0 :
        if init[i] > init[j]: total_init += 1

      if goal[i] > 0 and goal[j] > 0 :
        if goal[i] > goal[j]: total_goal += 1

  if total_init%2 == total_goal%2: return True
  else: return False

hasSolution(startNode, finalNode)


# Berikut adalah eksekusi algoritmanya

# In[114]:


# Mencetak node dan status node
def printNode(node):
    print(node[0],node[1],node[2])
    print(node[3],node[4],node[5])
    print(node[6],node[7],node[8])
    global nodeNumber
    print('Node:', nodeNumber)
    print('Depth:', len(node[9:]))
    print('Moves:', node[9:])
    print('------')
    nodeNumber += 1

# Cek kesesuaian state dengan goal state
def checkFinal(node):
    if node[:9]==finalNode:
        printNode(node)
        return True
    if node[:9] not in visitedList:
        printNode(node)
        nodeList.append(node)
        visitedList.append(node[:9])
    return False

# menghitung h(n) tergadap goal state
def calculateHeuristic(curr, goal):
    distance = 0
    indeks = 0
    for current in curr:
        target = goal.index(current)
        currentRow = int(indeks/3)
        currentColumn = indeks%3
        targetRow = int(target/3)
        targetColumn = target%3
        distance += abs(currentRow-targetRow) + abs(currentColumn-targetColumn)
        indeks += 1
    return distance

if __name__ == '__main__':
  # deklarasi variabel yang dibutuhkan  
    found = False
    nodeNumber = 0
    visitedList = []
    nodeList = []
    nodeList.append(startNode)
    visitedList.append(startNode)
    printNode(startNode)
    t0 = time.time()

    while (not found and not len(nodeList)==0):
      # Memilih node yang path cost-nya paling kecil
        fList = []
        for node in nodeList:
            h = calculateHeuristic(node[:9], finalNode)
            f = h
            fList.append(f)
        currentNode = nodeList.pop(fList.index(min(fList)))

        # ekspansi dan menambahkan node ke dalam Queue, serta memeriksa kesesuaiannya
        # dengan goal state

        #Ekspansi ke atas
        blankIndex = currentNode.index(0)
        if blankIndex!=0 and blankIndex!=1 and blankIndex!=2:
            upNode = copy.deepcopy(currentNode)
            upNode[blankIndex] = upNode[blankIndex-3]
            upNode[blankIndex-3] = 0
            upNode.append('up')
            found = checkFinal(upNode)

        #Ekspansi ke kiri
        if blankIndex!=0 and blankIndex!=3 and blankIndex!=6 and found==False:
            leftNode = copy.deepcopy(currentNode)
            leftNode[blankIndex] = leftNode[blankIndex-1]
            leftNode[blankIndex-1] = 0
            leftNode.append('left')
            found = checkFinal(leftNode)

        #Ekspansi ke bawah
        if blankIndex!=6 and blankIndex!=7 and blankIndex!=8 and found==False:
            downNode = copy.deepcopy(currentNode)
            downNode[blankIndex] = downNode[blankIndex+3]
            downNode[blankIndex+3] = 0
            downNode.append('down')
            found = checkFinal(downNode)

        #Ekspansi ke kanan
        if blankIndex!=2 and blankIndex!=5 and blankIndex!=8 and found==False:
            rightNode = copy.deepcopy(currentNode)
            rightNode[blankIndex] = rightNode[blankIndex+1]
            rightNode[blankIndex+1] = 0
            rightNode.append('right')
            found = checkFinal(rightNode)

    # Pencatatan waktu selesai
    t1 = time.time()
    print('Time:', t1-t0)
    print('------')

