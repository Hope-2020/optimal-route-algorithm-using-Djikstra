import sys 
  
class Graph(): 
  
    def __init__(self, vertices): 
        self.V = vertices 
        self.graph = [[0 for column in range(vertices)]  
                    for row in range(vertices)] 
  
    def printSolution(self, src, dist, dest): 

        print ("Forwarding Table for ", src+1)
        print("{:>10} {:>10} {:>10}".format("To", "Cost", "Next Hop"))
        for node in dest: 

            # calculate the shortest_path
            shortest_path = []
            end = node - 1
            while end is not None:
                shortest_path.append(end)
                end = parents[end]
            shortest_path.reverse()
            
            # print the result
            if len(shortest_path) > 1:
                print("{:>10} {:>10} {:>10}".format(node, dist[node-1], shortest_path[1]+1))
            else:
                print("{:>10} {:>10} {:>10}".format(node, -1, -1))
  
    # A utility function to find the vertex with  
    # minimum distance value, from the set of vertices  
    # not yet included in shortest path tree 
    def minDistance(self, dist, sptSet):

        min_index = -10
        # Initilaize minimum distance for next node 
        min = sys.maxsize 
  
        # Search not nearest vertex not in the  
        # shortest path tree 
        for v in range(self.V): 
            if (min_index < 0 or dist[v] < min) and sptSet[v] == False: 
                min = dist[v] 
                min_index = v 
  
        return min_index 
  
    # Funtion that implements Dijkstra's single source  
    # shortest path algorithm for a graph represented  
    # using adjacency matrix representation 
    def dijkstra(self, dest):

        #calculate the source list according the destination list 
        src_list = []
        for i in range(self.V):
            if i+1 not in dest:
                src_list.append(i)

        global parents
        for src in src_list:
            dist = [sys.maxsize] * self.V 
            dist[src] = 0
            sptSet = [False] * self.V 
            parents = [None] * self.V
    
            for cout in range(self.V): 
    
                # Pick the minimum distance vertex from  
                # the set of vertices not yet processed.  
                # u is always equal to src in first iteration 
                u = self.minDistance(dist, sptSet) 
    
                # Put the minimum distance vertex in the  
                # shotest path tree 
                sptSet[u] = True
    
                # Update dist value of the adjacent vertices  
                # of the picked vertex only if the current  
                # distance is greater than new distance and 
                # the vertex in not in the shotest path tree 
                for v in range(self.V): 
                    if self.graph[u][v] > 0 and sptSet[v] == False and dist[v] > dist[u] + self.graph[u][v]: 
                        dist[v] = dist[u] + self.graph[u][v] 
                        parents[v] = u
                        
            self.printSolution(src, dist, dest) 
  
 
if __name__ == "__main__":

    #input the data
    vetex_count = int(input("input the count of vetex: "))
    matrix = []
    print("input the matrix ")
    for i in range(vetex_count):
        row = list(map(int, input().split(' ')))
        matrix.append(row)
    dest = list(map(int, input("input the destination: ").split(' ')))

g = Graph(vetex_count) 
g.graph = matrix  
g.dijkstra(dest)