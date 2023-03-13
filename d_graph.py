# Author: Clint Lohr
# Assignment 6: Directed Graph
# Description: A class to implement a directed graph. Vertices and their connected vertices are stored using an
# adjacency matrix in the form of a list. The class contains methods for adding and removing edges and vertexes,
# checking the graph for valid paths, cycles, performs depth-first searches and breadth-first, and performs
# Dijkstra's Algorithm to find the shortest cost/distance path in the graph.
# searches.


import heapq
from collections import deque


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
        A method for adding a new vertex to the graph. Assigns vertex a reference index as an integer with the first
        vertex assigned index 0. Returns the number of vertices in the graph.
        """

        self.v_count += 1

        self.adj_matrix.append([0] * self.v_count)

        for index in range(self.v_count - 1):
            self.adj_matrix[index].append(0)

        count = self.v_count
        return count

    def add_edge(self, src: int, dst: int, weight=1) -> None:
        """
        A method for adding a new edge to the graph, connecting two vertices with provided indices. Does nothing if
        the src,  dst, or weight arguments are not valid.
        """

        if src >= self.v_count or dst >= self.v_count:
            return

        if src == dst:
            return

        if weight < 1:
            return

        self.adj_matrix[src][dst] = weight

    def remove_edge(self, src: int, dst: int) -> None:
        """
        A method for removing the edge between two vertices in the graph with provided indices. Does nothing if either
        of the index arguments are not valid.
        """
        if src < 0 or dst < 0:
            return

        if src >= self.v_count or dst >= self.v_count:
            return

        self.adj_matrix[src][dst] = 0

    def get_vertices(self) -> []:
        """
        A method to get a list of all vertices in the graph. Returns a list of the vertices.
        """

        vert_list = []

        for vert in range(self.v_count):
            vert_list.append(vert)

        return vert_list

    def get_edges(self) -> []:
        """
        A method to get a list of all edges in the graph. Returns a tuple with the two indices and the weight of the
        edge.
        """

        edge_list = []

        # iterates through each row of the matrix
        for row in range(self.v_count):
            length = len(self.adj_matrix[row])

            # iterates through each column of each row in the matrix
            for col in range(length):
                if self.adj_matrix[row][col] > 0:
                    edge_list.append((row, col, self.adj_matrix[row][col]))

        return edge_list

    def is_valid_path(self, path: []) -> bool:
        """
        A method for determining if a valid path exists in the graph. If the last vertex in the path list can be reached
        from the first vertex in the path list, the method returns True, or false otherwise. Takes as an argument
        a list of vertices.
        """

        path_len = len(path)

        if path_len == 0:
            return True

        # looks at the current and next vertex in the path list and checks if there is an edge between them.
        for index in range(path_len - 1):
            vert = path[index]
            next_vert = path[index + 1]
            if self.adj_matrix[vert][next_vert] == 0:
                return False

        return True

    def dfs(self, v_start, v_end=None) -> []:
        """
        A method that performs a depth-first search in the graph, keeping track of the vertices visited during the
        search in the order they were visited. Takes as a parameter a starting vertex and an optional ending vertex.
        If no ending vertex is provided, the search continues until all vertices are visited. Returns visited vertices
        as a list.
        """

        # stores a list of visited vertices from the starting vertex
        reachable = []
        verts = self.get_vertices()

        # boolean that will have a value of true if an end vertex was provided
        end = False

        if v_start not in verts:
            return reachable

        if v_end is not None:
            if v_end in verts:
                end = True

        # a deque to be used as a stack for storing all paths from the current vertex
        dfs_deq = deque()
        dfs_deq.append(v_start)

        while len(dfs_deq) > 0:
            # stores successors of the current vertex
            sort_list = []
            vert = dfs_deq.pop()
            length = self.adj_matrix[vert]
            vert_len = len(length)

            if vert not in reachable:
                reachable.append(vert)

                # checks if we have reached the end vertex, if provided
                if vert == v_end and end is True:
                    return reachable

                # iterates through the values of the current vertex to find its successors.
                for col in range(vert_len):
                    if self.adj_matrix[vert][col] > 0:
                        sort_list.append(col)

                    # puts sort list items in order to dequeue vertex names in ascending lexicographical order
                    sort_list.sort(reverse=True)
                    for j in sort_list:
                        dfs_deq.append(j)

        return reachable

    def bfs(self, v_start, v_end=None) -> []:
        """
        A method that performs a breadth-first search in the graph, keeping track of the vertices visited during the
        search in the order they were visited. Takes as a parameter a starting vertex and an optional ending vertex.
        If no ending vertex is provided, the search continues until all vertices are visited. Returns visited vertices
        as a list.
        """

        reachable = []
        verts = self.get_vertices()
        end = False

        if v_start not in verts:
            return reachable

        if v_end is not None:
            if v_end in verts:
                end = True

        # a deque to be used as a queue for storing all paths from the current vertex
        bfs_deq = deque()
        bfs_deq.appendleft(v_start)

        while len(bfs_deq) > 0:
            sort_list = []
            vert = bfs_deq.pop()
            length = self.adj_matrix[vert]
            vert_len = len(length)

            if vert not in reachable:
                reachable.append(vert)
                if vert == v_end and end is True:
                    return reachable
                for col in range(vert_len):
                    if self.adj_matrix[vert][col] > 0:
                        sort_list.append(col)
                    sort_list.sort()
                    for j in sort_list:
                        bfs_deq.appendleft(j)

        return reachable

    def has_cycle(self):
        """
        A method for determining if the graph has at least one cycle. Returns True if the graph has a cycle, or false
        otherwise.

        Cite: https://www.baeldung.com/cs/detecting-cycles-in-directed-graph : Drew upon parts of their their pseudocode
        for the recursive implementation. Used provided flowcharts to structure my code.

        Cite: https://www.youtube.com/watch?v=HDUzBEG1GlA&ab_channel=JoeJames from timestamp 2:18 to 9:12
        """

        # keeps two dictionaries with the vertices of the graph for the row, and the vertices of the graph for the
        # column. Values are False until vertex is visited.

        verts = self.get_vertices()
        visit_row = {}
        visit_col = {}

        for vert in verts:
            visit_row[vert] = False
            visit_col[vert] = False

        # iterates through each row of the matrix, moves to the next vertex if current vertex has already been visited.
        for vert in verts:
            if visit_row[vert] is False:
                cycle = self.rec_has_cycle(vert, visit_row, visit_col)
                if cycle is True:
                    return True

        return False

    def rec_has_cycle(self, vert, row, col):
        """
        Helper method for the 'has_cycle' method. Takes as arguments the current vertex, the visited rows, and the
        visited columns.
        """

        # sets the current row/column to True since we are currently visiting it.
        row[vert] = True
        col[vert] = True
        length = len(self.adj_matrix)

        # iterates through each column of the current row of the matrix
        for index in range(length):
            # checks if an edge exists from the current vertex to its successors. If the weight is greater than zero,
            # an edge exists.
            if self.adj_matrix[vert][index] != 0:
                if row[index] is False:

                    # calls the recursive method with the current row, col and a successor of the current vertex.
                    cycle = self.rec_has_cycle(index, row, col)

                    # if a vertex has been reached that has been visited before and that vertex got there by a path
                    # that wasn't from being an adjacent vertex.
                    if cycle is True:

                        return True

                # If a row and a col at the given index has been previously visited a cycle has been found.
                elif col[index] is True:

                    return True

        # if no cycles were found, col[vert] is reverted back to False
        col[vert] = False

        return False


    def dijkstra(self, src: int) -> []:
        """
        A method to implement Dijkstra's Algorithm using a directed adjacency matrix. The algorithm computes the path
        in the graph with shortest cost/distance from a given vertex to all other vertices in the graph. If a vertex
        is not reachable, the value equals infinity. Takes as a parameter a starting vertex. Returns a list of the
        total cost of travel from the starting vertex to each of the vertices in the graph individually.
        """

        # KEY = VERTEX V, VALUE = MIN DISTANCE D TO VERTEX
        visited = {}
        p_queue = []

        # creates a tuple with cost and starting vertex. Initialized to zero for the src vertex.
        source = (0, src)

        # Uses a heapq to store and maintain a min heap for the vertices to be travelled.
        heapq.heappush(p_queue, source)

        while len(p_queue) > 0:

            dist, vert = heapq.heappop(p_queue)

            # adds vert to the visited map with its distance.
            if vert not in visited:
                visited[vert] = dist

                # checks each column in the current row for weights. Any value greater than zero signifies an edge.
                for col in range(self.v_count):
                    if self.adj_matrix[vert][col] > 0:
                        distance = self.adj_matrix[vert][col]

                        # adds the total distance travelled thus far, to the distance between current and its successor.
                        succ_dist = distance + dist
                        heapq.heappush(p_queue, (succ_dist, col))

        # Replaces unvisited vertices in the graph with the value 'infinity'
        for i in range(self.v_count):
            if i not in visited:
                visited[i] = float('inf')

        # sorts the keys in visited so indices match
        sort_keys = {}
        for i in sorted(visited):
            sort_keys[i] = visited[i]

        # creates a list with just the cost/distance
        len_list = []
        for i in sort_keys:
            len_list.append(visited[i])

        return len_list
