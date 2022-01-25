#!/usr/bin/env python
# coding: utf-8

# In[12]:


#BUILDING MATRIX                                                 

Graph=[[0,80,139,31.1,999,999,999,999,999,999,999,97.5,999],
[80,0,999,999,999,999,999,999,999,999,999,999,75.4],
[139,999,0,999,999,999,999,999,999,999,39,999,999],
[31.1,999,999,0,51.7,999,999,999,999,51.7,999,999],
[999,999,999,51.7,0,28.8,999,999,999,999,999,999,999],
[999,999,999,999,28.8,0,47.7,999,999,999,999,999,999],
[999,999,999,999,999,47.7,0,38,999,999,999,999,999],
[999,999,999,999,999,999,38,0,109,999,26,999,100],
[999,999,999,999,999,999,999,103,0,22,999,999,999],
[99,999,999,51.7,999,999,999,999,22,0,999,999,999],
[999,999,39,999,999,999,999,26,999,999,0,999,999],
[97.5,999,999,999,999,999,999,999,999,999,0,70.4],
[999,75.4,999,999,999,999,999,100,999,999,999,70.4,0]]


# In[13]:


#Dijsktra's Algorithm                                          

class GRAPH():
	def __init__(self, vertices):
		self.V = vertices
		self.graph = [[0 for column in range(vertices)]
					for row in range(vertices)]
	def printSolution(self, dist):
		print("Vertex \tDistance from Source")
		for node in range(self.V):
			print(node+1, "\t", round(dist[node],2))
	def minDistance(self, dist, sptSet):
		min = 100000
		for u in range(self.V):
			if dist[u] < min and sptSet[u] == False:
				min = dist[u]
				min_index = u
		return min_index
	def dijkstra(self, src):
		dist = [100000] * self.V
		dist[src] = 0
		sptSet = [False] * self.V
		for cout in range(self.V):
			x = self.minDistance(dist, sptSet)
			sptSet[x] = True
			for y in range(self.V):
				if self.graph[x][y] > 0 and sptSet[y] == False and 				dist[y] > dist[x] + self.graph[x][y]:
						dist[y] = dist[x] + self.graph[x][y]
		self.printSolution(dist)
g = GRAPH(12)
g.graph = Graph
g.dijkstra(0);

#SOURCE~> Belgaum (1)
#DESTINATION~> Karwar (8)


# In[14]:


#Floyd Warshal's Algorithm                     
INF=999
V = len(Graph)-1
def floydWarshall(graph,m,n):

    dist=graph
    for k in range(V):
        for i in range(V):
            for j in range(V):
                dist[i][j]=round(min(dist[i][j],dist[i][k]+dist[k][j]),2)
    source=m;dest=n
    print("{} ~>{} is => {}".format(m+1,n+1,dist[n][m]))
floydWarshall(Graph,0,7)

#SOURCE~> Belgaum (1)
#DESTINATION~> Karwar (8)


# In[15]:


# Bellman Ford Algorithm                         


class BGraph:
    def __init__(self, vertices):
        self.V = vertices   
        self.graph = []     
    # Add edges
    def add_edge(self, s, d, w):
        self.graph.append([s, d, w])

    def print_solution(self, dist):
        print("Vertex Distance from Source")
        for i in range(self.V):
            print("{0}\t\t{1}".format(i, dist[i]))
    def bellman_ford(self, src):
        dist = [999] * self.V
        dist[src] = 0
        for _ in range(self.V - 1):
            for s, d, w in self.graph:
                if dist[s] != 999 and dist[s] + w < dist[d]:
                    dist[d] = dist[s] + w
        for s, d, w in self.graph:
            if dist[s] != 999 and dist[s] + w < dist[d]:
                print("Graph contains negative weight cycle")
                return
        self.print_solution(dist)
G=BGraph(13)
for i in range(13):
    for j in range(i+1,12):
        Q=Graph[i][j]
        G.add_edge(i,j,Q)
G.bellman_ford(0)

#SOURCE~> Belgaum (0)
#DESTINATION~> Karwar (7)

