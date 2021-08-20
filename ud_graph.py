# Course: Data Structures
# Author: Albert Chap
# Assignment: Undirected Graph
# Description: Implement the UndirectedGraph class by completing the provided skeleton code.
#              Designed to support the following type of graph: undirected, unweighted,
#              no duplicate edges, no loops. Cycles are allowed.


class UndirectedGraph:
    """
    Class to implement undirected graph
    - duplicate edges not allowed
    - loops not allowed
    - no edge weights
    - vertex names are strings
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency list
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.adj_list = dict()

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            for u, v in start_edges:
                self.add_edge(u, v)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        out = [f'{v}: {self.adj_list[v]}' for v in self.adj_list]
        out = '\n  '.join(out)
        if len(out) < 70:
            out = out.replace('\n  ', ', ')
            return f'GRAPH: {{{out}}}'
        return f'GRAPH: {{\n  {out}}}'

    # ------------------------------------------------------------------ #

    def add_vertex(self, v: str) -> None:
        """
        param: A vertex( string)
        returns:  None

        """
        if v in self.adj_list:  #if vertex already exists, return
            return
        self.adj_list[v] = []



    def add_edge(self, u: str, v: str) -> None:
        """
        Add edge to the graph
        """
        if u == v:
            return
        #if edge exists, return
        elif u in self.adj_list and v in self.adj_list[u]:
            return
        # add both vertices to the dictionary and connect them
        else:
            self.add_vertex(u)
            self.add_vertex(v)
            self.adj_list[u].append(v)
            self.adj_list[v].append(u)

        

    def remove_edge(self, v: str, u: str) -> None:
        """
        Remove edge from the graph
        """
        #if both vertices do not exit return
        if v not in self.adj_list or u not in self.adj_list:
            return
        # if edge does not exist, do nothing
        elif u not in self.adj_list[v]:
            return
        else:   # remove the connection
            self.adj_list[v].remove(u)
            self.adj_list[u].remove(v)

    def remove_vertex(self, v: str) -> None:
        """
        Remove vertex and all connected edges
        """
        # if the vertex doesn't exist, return
        if v not in self.adj_list:
            return
        for vertex in self.adj_list[v]:         # remove the edges connected to that vertex
            self.adj_list[vertex].remove(v)
        del self.adj_list[v]
        

    def get_vertices(self) -> []:
        """
        Return list of vertices in the graph (any order)
        """
        vertices = []
        for v in self.adj_list:
            vertices.append(v)
        return vertices

    def get_edges(self) -> []:
        """
        Return list of edges in the graph (any order)
        """
        edges = []
        visited = []    # visited vertices
        for i in self.adj_list:
            for j in self.adj_list[i]:
                if j not in visited:
                    edges.append((i,j))
            visited.append(i)
        return edges

    def is_valid_path(self, path: []) -> bool:
        """
        Return true if provided path is valid, False otherwise
        """
        #if only one vertex
        if len(path) == 1:
            if path[0] in self.get_vertices():
                return True
            else:
                return False
        elif len(path) == 0: # empty path return true
            return True
        for v in range(1,len(path)):
            if path[v] not in self.adj_list[path[v-1]]:
                return False
        return True

        # empty path is valid return True.

    def dfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during DFS search
        Vertices are picked in alphabetical order
        """
        dfsPath = []
        if v_start not in self.get_vertices():
            return dfsPath
        else:
            stack = [v_start]
        while stack:
            current = stack[-1]   # first value of the stack
            if current not in dfsPath:   # if current not in the path append the value
                dfsPath.append(current)
            visited = True              # set indicator for visited vertex to True
            if current == v_end:
                return dfsPath
            newList = self.adj_list[current]
            newList.sort()
            for v in newList:
                if v not in dfsPath:
                    stack.append(v)
                    visited = False  # set visited flag to false when encountering another vertex
                    break
            if visited == True: # if vert
                stack.pop()
        return dfsPath

    def bfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during BFS search
        Vertices are picked in alphabetical order
        """
        visited = []
        queue = []
        if v_start not in self.get_vertices():
            return visited
        else:
            visited.append(v_start)
            queue.append(v_start)

        while queue:
            s = queue.pop(0)
            if s == v_end:
                return visited
            list = self.adj_list[s]
            list.sort()
            for neighbour in list:
                if neighbour == v_end:
                    visited.append(neighbour)
                    return visited
                if neighbour not in visited:
                    visited.append(neighbour)
                    queue.append(neighbour)
        return visited

    def count_connected_components(self):
        """
        Return number of connected componets in the graph
        """
        visited = []
        count = 0
        for v in self.adj_list :
            if v not in visited :
                for v2 in self.dfs(v) :
                    visited.append(v2)
                count += 1
        return count

    def has_cycle(self):
        """
        Return True if graph contains a cycle, False otherwise
        """
        for v_start in self.adj_list :
            path = []
            stack = [v_start]
            while stack:
                current = stack[-1]
                if current not in path :
                    path.append(current)
                # set flag for visited vertex
                visited = True
                # sort the neighbor indices to make sure we get right one
                lst = self.adj_list[current]
                lst.sort()
                for v in lst:
                    if v == v_start and v in path:
                        return True
                    if v not in path:
                        stack.append(v)
                        visited = False
                        break
                if visited:
                    stack.pop()
        return False



if __name__ == '__main__':

    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = UndirectedGraph()
    print(g)

    for v in 'ABCDE':
        g.add_vertex(v)
    print(g)

    g.add_vertex('A')
    print(g)

    for u, v in ['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE', ('B', 'C')]:
        g.add_edge(u, v)
    print(g)


    print("\nPDF - method remove_edge() / remove_vertex example 1")
    print("----------------------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    g.remove_vertex('DOES NOT EXIST')
    g.remove_edge('A', 'B')
    g.remove_edge('X', 'B')
    print(g)
    g.remove_vertex('D')
    print(g)


    print("\nPDF - method get_vertices() / get_edges() example 1")
    print("---------------------------------------------------")
    g = UndirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE'])
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    test_cases = ['ABC', 'ADE', 'ECABDCBE', 'ACDECB', '', 'D', 'Z']
    for path in test_cases:
        print(list(path), g.is_valid_path(list(path)))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = 'ABCDEGH'
    for case in test_cases:
        print(f'{case} DFS:{g.dfs(case)} BFS:{g.bfs(case)}')
    print('-----')
    for i in range(1, len(test_cases)):
        v1, v2 = test_cases[i], test_cases[-1 - i]
        print(f'{v1}-{v2} DFS:{g.dfs(v1, v2)} BFS:{g.bfs(v1, v2)}')

    print("\nPDF - method count_connected_components() example 1")
    print("---------------------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print(g.count_connected_components(), end=' ')
    print()


    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG',
        'add FG', 'remove GE')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print('{:<10}'.format(case), g.has_cycle())
