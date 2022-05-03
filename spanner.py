import sys
import argparse
import re
import networkx as nx
from networkx.algorithms.shortest_paths.weighted import dijkstra_path_length
from networkx.algorithms.shortest_paths.weighted import dijkstra_path
import matplotlib.pyplot as plt
import math
from itertools import islice

graph = {}

E = {}


def connectivity(H):
    for u in range(1,13):
        for v in range(1,13):
            if u != v:
                #check that there are 3 paths less than 130 in the complete graph
                ignore = []
                paths_found = True
                direct=False
                for p in range(0,3):
                    
                    length,path = dijkstras(graph,u,v,ignore,direct)
                    #test with 20ms difference
                    if length > 110:
                        paths_found=False
                        break
                    for node in path:
                        if node != u and node != v:
                            ignore.append(node)
                        if len(path) == 2:
                            direct=True
                            
                #if the complete graph containts 3 disjoint paths verify that the spanner does
                if paths_found == True:
                    ignore = []
                    direct=False
                    for p in range(0,3):
                        length,path = dijkstras(H,u,v,ignore,direct)
                        if path == None:
                            print("spanner failed for u = " + str(u) + " v = " + str(v))
                            break
                        
                        if length > 130:
                            print("spanner failed for u = " + str(u) + " v = " + str(v))
                            
                            #break
                        for node in path:
                            if node != u and node != v:
                                ignore.append(node)
                            if len(path) == 2:
                                direct=True
                                
#runs dijkstras with a set of nodes to ignore                         
def dijkstras(H, u, v, ignore, direct):
    best = nx.DiGraph()
    for n in H:
        for m in H[n]:
            if m not in ignore and n not in ignore:
                if not(n==u and m==v and direct==True) and not(m==u and n==v and direct== True):
                    weight = H[n][m]
                    best.add_edge(n, m, weight=weight)
                    best.add_edge(m, n, weight=weight)
                    
    try:
        length = dijkstra_path_length(G=best, source=u, target=v, weight="weight")
    except nx.NetworkXNoPath:
        return 0,None
    except nx.NodeNotFound:
        return 0,None
    path = dijkstra_path(G=best, source=u, target=v, weight="weight")
    return length,path

    

def LBC(t, a, u, v,H,mode):
    ignore = []
    direct=False
    #run either bfs and dijkstras a set number of times
    for i in range(1, a+1):
        if mode == 0:
            result, value = BFS(u,v,t,ignore,H)
        else:
            value, result = dijkstras(H,u,v,ignore,direct)
        if result is None or value > t:
            return True
        else:
            #adding nodes in path to f
            
            #add nodes into the ignore set
            for node in result:
                
                if node != u and node != v:
                    ignore.append(node)
                if len(result) == 2:
                    ignore.append("direct")
                    direct = True
    return False

def greedy(k, f,H,mode):
    count = 0
    #order the edges in terms of length
    for edge in sorted(E.items(), key=lambda x: x[1]):
        edge=edge[0]
        ret = LBC(k*graph[edge[0]][edge[1]], f, edge[0], edge[1],H,mode)
        
        #if lbc returns true add the edge to H
        if ret == True:
            
            H[edge[0]][edge[1]] = graph[edge[0]][edge[1]]
            H[edge[1]][edge[0]] = graph[edge[1]][edge[0]]
            count = count + 1
    return count

#breadth first search with a set of nodes to ignore
def BFS(start, goal,t,ignore,H):
    explored = []
    queue = [[start]]
    value = {}
    for a in range(1, len(graph)+1):
            value[a]=0
    while queue:
        path = queue.pop(0)
        node = path[-1]
        
        # Condition to check if the
        # current node is not visited
        if node not in explored:
            neighbours = H[node]
             
            # Loop to iterate over the
            # neighbours of the node
            for neighbour in neighbours:
                #print(graph[node][neighbour])
                #print(t)
                if value[node] + graph[node][neighbour] <= t and neighbour not in ignore:
                    if not("direct" in ignore and neighbour == goal and node == start):
                        new_path = list(path)
                        new_path.append(neighbour)
                        queue.append(new_path)
                        value[neighbour]= value[node] + graph[node][neighbour]
                         
                        # Condition to check if the
                        # neighbour node is the goal
                        if neighbour == goal:
                            return (new_path, value[neighbour])
            explored.append(node)
 
    # Condition when the nodes
    # are not connected
    return None, 0
    
    
def read_positions(pos_file):
    """ Read node positions from the given pos_file. Return a dictionary that
    maps each node in the file to a tuple (x,y) with its x,y coordinates """
    pos = {}
    with open(pos_file) as f:
        for line in f:
            # each line is like:     1 41.505880 -81.609169 # Case Western
            parts = line.split()
            node = int(parts[0])
            lat = float(parts[1])
            lon = float(parts[2])
            pos[node] = (lon, lat)
    return pos
    
def create_graph(input_graph):
    with open(input_graph) as f:
        for line in f:
            parts = line.split()
            if parts != []:
                node1 = int(parts[0])
                node2 = int(parts[1])
                weight = int(parts[2])
                E[(node1,node2)] = int(weight)
                if not(node1 in graph):
                    graph[node1]={}
                graph[node1][node2] = int(weight)

    
def compute_metrics(H, k):
    connectivity(H)
    
    total = 0
    cnt = 0
    for node in H:
        for edge in H[node]:
            cnt = cnt + 1
            total = total + H[node][edge]
    average = total/cnt
    prev = 0
    num = 0
    for node in graph:
        for edge in graph[node]:
            num = num+1
            prev = prev + graph[node][edge]
    avg = prev/num
    print("edge count = " + str(cnt/2))
    print("average hop distance " + str(average))
    print()
    
def main(argv):
    inputGraph = argv[0]
    posGraph = argv[1]
    kStart = float(argv[2])
    kTries = int(argv[3])
    mode = argv[4]
    
    create_graph(inputGraph)
    
    pos = read_positions(posGraph)
    
    if mode == 'b':
        mode = 0
    elif mode == 'd':
        mode = 1
    

    k = kStart
    H = {}
    
    #run on different values of k
    for x in range(0, int(kTries)):
        for a in range(1, len(graph)+1):
            H[a]={}
        print("k value = " + str(k))
        # 3 here refers to the number of disjoint paths to find
        greedy(k,3,H,mode)
        
        
        compute_metrics(H, k)
        
        
        pos = read_positions("12node_pos.txt")
        
        
        #draws the graph from position graph
        draw = nx.DiGraph()
        for n in H:
            for m in H[n]:
                weight = H[n][m]
                draw.add_edge(n, m, weight=weight)
                draw.add_edge(m, n, weight=weight)
        nx.draw_networkx(draw, pos)
        plt.show()
        
        
        #write the spanner to file
        if mode == 0:
            with open("bfs"+str(round(k, 1))+inputGraph+".txt", 'w') as f:
                for u in H:
                    for v in H[u]:
                        f.write(str(u) + " " + str(v) + " " + str(H[u][v]) + "\n")
        else if mode == 1:
            with open("dijk"+str(round(k, 1))+inputGraph+".txt", 'w') as f:
                for u in H:
                    for v in H[u]:
                        f.write(str(u) + " " + str(v) + " " + str(H[u][v]) + "\n")
                    
        
        #update the k value
        k=k+0.1
        k = round(k,1)
if __name__ == "__main__":
    main(sys.argv[1:])



















    