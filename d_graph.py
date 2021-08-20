# Course: CS261 - Data Structures
# Author: Albert Chap
# Assignment: Directed Graph
# Description: Implement the DirectedGraph class by completing the provided skeleton code.
#              Designed to support the following type of graph: directed,
#              weighted (positive only), no duplicate edges, no loops.

class DirectedGraph:
    """
    Class to implement directed weighted graph
    - duplicate edges not allowed
    - loops not allowed
    - only positive edge weights
    - vertex names are integers
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency matrix
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.v_count = 0
        self.adj_matrix = []

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            v_count = 0
            for u, v, _ in start_edges:
                v_count = max(v_count, u, v)
            for _ in range(v_count + 1):
                self.add_vertex()
            for u, v, weight in start_edges:
                self.add_edge(u, v, weight)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        if self.v_count == 0:
            return 'EMPTY GRAPH\n'
        out = '   |'
        out += ' '.join(['{:2}'.format(i) for i in range(self.v_count)]) + '\n'
        out += '-' * (self.v_count * 3 + 3) + '\n'
        for i in range(self.v_count):
            row = self.adj_matrix[i]
            out += '{:2} |'.format(i)
            out += ' '.join(['{:2}'.format(w) for w in row]) + '\n'
        out = f"GRAPH ({self.v_count} vertices):\n{out}"
        return out

    # ------------------------------------------------------------------ #

    def add_vertex(self) -> int:
        """
        Adds a new vertex to the graph, which will be assigned a reference index.
        """
        self.v_count += 1
        for list in self.adj_matrix:
            list.append(0)
        self.adj_matrix.append([0 for _ in range(self.v_count)])
        return self.v_count

    def add_edge(self, src: int, dst: int, weight=1) -> None:
        """
        Adds a new edge to the graph, connecting two vertices with provided indices.
        If the edge already exists, update the weight of that edge.
        """
        if src not in range(self.v_count) or dst not in range(self.v_count) :
            return
        elif weight < 0 :
            return
        elif src == dst :
            return
        else :
            lsts = self.adj_matrix
            lsts[src][dst] = weight

    def remove_edge(self, src: int, dst: int) -> None:
        """
        Removes an edge between two vertices.
        """
        if src not in range(self.v_count) or dst not in range(self.v_count) :
            return
        lsts = self.adj_matrix
        lsts[src][dst] = 0

    def get_vertices(self) -> []:
        """
        Returns a list of vertices in the graph. Order does not matter.
        """
        res = []
        for v in range(self.v_count) :
            res.append(v)
        return res

    def get_edges(self) -> []:
        """
        Returns a list of edges in the graph. Order does not matter.
        """
        res = []
        lsts = self.adj_matrix
        for src in range(self.v_count) :
            for dst in range(len(lsts[src])):
                if lsts[src][dst] :
                    res.append((src, dst, lsts[src][dst]))
        return res

    def is_valid_path(self, path: []) -> bool:
        """
        Returns True if the sequence of vertices represents a valid path.
        """
        if path == [] :  # empty path is valid
            return True
        elif len(path) == 1 :  # if only 1 vertex
            if path[0] in self.get_vertices() :
                return True
            return False
        else :
            lsts = self.adj_matrix
            # check first 2 vertices
            if not lsts[path[0]][path[1]] :
                return False
            for i in range(1, len(path)) :
                if not lsts[path[i - 1]][path[i]] :
                    return False
            return True

    def dfs(self, v_start, v_end=None) -> []:
        """
        Performs a depth-first search (DFS) in the graph and
        returns a list of vertices visited during the search.
        """
        if v_start not in self.get_vertices() :
            return []
        visited = [v_start]
        # use recursive function to dps through each found vertex
        return self.searchVertex(v_start, visited, v_end)

    def searchVertex(self, v, visited, v_end) :
        """
        recursive function to dfs through each found vertex
        """
        lsts = self.adj_matrix
        if v not in visited:
            visited.append(v)
        if v == v_end:
            return visited
        for i in range(self.v_count) :
            if lsts[v][i] > 0 and i not in visited :
                self.searchVertex(i, visited, v_end)
        return visited


    def bfs(self, v_start, v_end=None) -> []:
        """
        Performs a breadth first search (BFS) in the graph and
        returns a list of vertices visited during the search.
        """

        if v_start not in self.get_vertices():
            return []
        visited = [False] * self.v_count
        bfs_list = []
        queue = []
        visited[v_start] = True
        queue.append(v_start)
        while len(queue) != 0:
            u = queue.pop(0)
            bfs_list.append(u)
            if u == v_end:
                return bfs_list
            for i in range(self.v_count):
                if visited[i] == False and self.adj_matrix[u][i] > 0:
                    queue.append(i)
                    visited[i] = True
        return bfs_list


    def has_cycle(self):
        """
        Returns a list of vertices visited during the search, in the order they were visited.
        (Depth-first search)
        """
        listofVertex = self.get_vertices()
        start_vertex = listofVertex[0]
        # flags for indication -1 = unvisited, 0 = visited and in stack, 1 visted and popped out from stack
        flagList = [-1 for _ in range(self.v_count)]
        #determines if there is a cycle
        isCycle = False
        parentMap = {}
        visited = []
        stack = [start_vertex]
        while stack and isCycle == False:
            counter = 0
            s = stack[-1]
            if s not in visited:
                visited.append(s)
                flagList[s] = 0
            for v in range(self.v_count):
                counter += 1     # increment counter
                # if flag is unvisted and value is greater than 0
                if flagList[v] == -1 and self.adj_matrix[s][v] > 0:
                    stack.append(v)
                    visited.append(v)
                    parentMap[s] = v
                    flagList[v] = 0
                    break
                # if it is the end of the map pop the stack and set flag to 1
                elif self.adj_matrix[s] == [0]* len(self.adj_matrix):
                    stack.pop()
                    flagList[s] = 1
                    break
                # if the value in matrix is > 0 and the flag is 0 then there exists a cycle
                elif flagList[v] == 0 and self.adj_matrix[s][v] > 0:
                    isCycle = True
                    break
                # if at the end of the loop there is no vertices left to visit pop that value and change flag
                elif counter == len(self.adj_matrix) and v in visited:
                    stack.pop()
                    flagList[s] = 1
                    break
        return isCycle




    def dijkstra(self, src: int) -> []:
        """
        Implements the Dijkstra algorithm to compute the length of the shortest path from
        a given vertex to all other vertices in the graph.
        """
        maxDistance = 2147483646
        dist = [maxDistance] * self.v_count
        dist[src] = 0
        shortestPath = [False] * self.v_count  # create a matrix to flag shortest path
        for i in range(self.v_count):
            # use min distance vertex from set of vertices
            u = self.getMinDistanceVertex(dist, shortestPath)
            # set min distance in shortest path array
            shortestPath[u] = True
            for v in range(self.v_count) :
                # if distance is > new distance and not in shortest path array update distance value
                if self.adj_matrix[u][v] > 0 and shortestPath[v] == False and dist[v] > (dist[u] + self.adj_matrix[u][v]):
                    dist[v] = dist[u] + self.adj_matrix[u][v]

        for i in range(len(dist)) :
            if dist[i] == maxDistance :
                dist[i] = float('inf')
        return dist

    def getMinDistanceVertex(self, dist, shortestPath):
        """Returns the minimum distance node. """
        min = float('inf')
        minIndex = 0
        for v in range(self.v_count):
            if shortestPath[v] == False and dist[v] < min :
                min = dist[v]
                minIndex = v
        return minIndex


if __name__ == '__main__':

    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = DirectedGraph()
    print(g)
    for _ in range(5):
        g.add_vertex()
    print(g)

    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    for src, dst, weight in edges:
        g.add_edge(src, dst, weight)
    print(g)


    print("\nPDF - method get_edges() example 1")
    print("----------------------------------")
    g = DirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    test_cases = [[0, 1, 4, 3], [1, 3, 2, 1], [0, 4], [4, 0], [], [2]]
    for path in test_cases:
        print(path, g.is_valid_path(path))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for start in range(5):
        print(f'{start} DFS:{g.dfs(start)} BFS:{g.bfs(start)}')


    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)

    edges_to_remove = [(3, 1), (4, 0), (3, 2)]
    for src, dst in edges_to_remove:
        g.remove_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')

    edges_to_add = [(4, 3), (2, 3), (1, 3), (4, 0)]
    for src, dst in edges_to_add:
        g.add_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')
    print('\n', g)


    print("\nPDF - dijkstra() example 1")
    print("--------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
    g.remove_edge(4, 3)
    print('\n', g)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
