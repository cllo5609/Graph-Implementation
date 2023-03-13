# Author: Clint Lohr
# Assignment 6: Undirected Graph
# Description: A class to implement an undirected graph. Vertices and their connected vertices are stored using an
# adjacency list in the form of a dictionary. The class contains methods for adding and removing edges and vertexes,
# checking the graph for valid paths, connected components, cycles, and performs depth-first searches and breadth-first
# searches.

import heapq
from collections import deque


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
        A method for adding a new vertex to the graph. Takes as a parameter a vertex to add as a string.
        """

        if v not in self.adj_list:
            self.adj_list[v] = []

    def add_edge(self, u: str, v: str) -> None:
        """
        A method for Adding an edge between two vertices to the graph. Takes as parameters two vertices to connect as
        strings.
        """
        if u == v:
            return

        # creates new vertex if it does not already exist in the list.
        if u not in self.adj_list:
            self.adj_list[u] = []

        if v not in self.adj_list:
            self.adj_list[v] = []

        # adds the vertex to the connecting vertex's list of edges
        if v not in self.adj_list[u]:
            self.adj_list[u].append(v)

        if u not in self.adj_list[v]:
            self.adj_list[v].append(u)

    def remove_edge(self, v: str, u: str) -> None:
        """
        A method for removing an edge between two vertices in the graph by removing the vertex from the appropriate
        key in the adjacency list. Takes as arguments the two vertices of the edge.
        """

        if v not in self.adj_list:
            return

        if u not in self.adj_list:
            return

        if u in self.adj_list[v]:
            # gets the number of values associated with key 'v'
            length = len(self.adj_list[v])
            # iterates through the values of 'v' until 'u' is found
            for index in range(length):
                value = self.adj_list[v][index]
                if value == u:
                    self.adj_list[v].pop(index)
                    break

        if v in self.adj_list[u]:
            length = len(self.adj_list[u])
            for index in range(length):
                value = self.adj_list[u][index]
                if value == v:
                    self.adj_list[u].pop(index)
                    break

    def remove_vertex(self, v: str) -> None:
        """
        A method to remove a given vertex from the graph by removing all edges connected to that vertex before poping
        the vertex from the adjacency list. Takes as an argument the vertex to be removed.
        """

        if v not in self.adj_list:
            return

        rem_len = len(self.adj_list[v])
        # iterates through all values of key 'v' in the adjacency list
        for index in range(rem_len):
            rem = self.adj_list[v][index]
            r_len = len(self.adj_list[rem])

            # iterates through all values of each key that was a value in key 'v' until 'v' is found. Removes 'v'
            for j in range(r_len):
                if self.adj_list[rem][j] == v:
                    self.adj_list[rem].pop(j)
                    break

        self.adj_list.pop(v)

    def get_vertices(self) -> []:
        """
        A method for getting all vertices of the graph without removing them. Returns a list of vertices.
        """

        key_list = []
        length = len(self.adj_list)
        if length == 0:
            return key_list

        deq = deque(self.adj_list)
        for index in range(length):
            key = deq[index]
            key_list.append(key)

        return key_list

    def get_edges(self) -> []:
        """
        A method for getting all edges in the graph without removing them. Returns a list of edges.
        """

        edge_list = []
        length = len(self.adj_list)
        if length == 0:
            return edge_list

        deq = deque(self.adj_list)
        deq_len = len(deq) - 1

        while deq_len >= 0:
            key = deq.popleft()
            key_len = len(self.adj_list[key])

            # iterates through each vertex in the graph and appends the vertex and each of its edges to a list.
            for index in range(key_len):
                edge = self.adj_list[key][index]

                # checks if edge is already in the list
                edge_check = (edge, key)
                if edge_check not in edge_list:
                    edge_list.append((key, edge))
            deq_len -= 1

        return edge_list

    def is_valid_path(self, path: []) -> bool:
        """
        A method the check if a list of provided vertices forms a valid path in the graph. Takes as an argument a list
        of vertex names. Returns True if the sequence of vertices represents a valid path or False otherwise. Empty path
        is considered valid.
        """

        path_len = len(path)
        if path_len == 0:
            return True

        if path_len == 1:
            if path[0] in self.adj_list:
                return True
            return False

        keys = self.get_vertices()
        for vert in path:
            if vert not in keys:
                return False

        # Used to iterate through the vertices in the path list
        for index in range(path_len - 1):
            vert = path[index]
            next_vert = path[index + 1]

            # checks if there is an edge between the current and next vertex from path by checking the adjacency list
            # of the current vertex
            if next_vert not in self.adj_list[vert]:
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
        keys = self.get_vertices()

        # boolean that will have a value of true if an end vertex was provided
        end = False

        if v_start not in keys:
            return reachable

        if v_end is not None:
            if v_end in keys:
                end = True

        # a deque to be used as a stack for storing all paths from the current vertex
        dfs_deq = deque()
        dfs_deq.append(v_start)

        while len(dfs_deq) > 0:

            # stores successors of the current vertex
            sort_list = []
            vert = dfs_deq.pop()
            vert_len = len(self.adj_list[vert])

            if vert not in reachable:
                reachable.append(vert)

                # checks if we have reached the end vertex, if provided
                if vert == v_end and end is True:
                    return reachable

                # iterates through the values of the current vertex to find its successors.
                for index in range(vert_len):
                    sort_list.append(self.adj_list[vert][index])

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
        keys = self.get_vertices()
        # boolean that will have a value of true if an end vertex was provided
        end = False

        if v_start not in keys:
            return reachable

        if v_end is not None:
            if v_end in keys:
                end = True

        # a deque to be used as a queue for storing all paths from the current vertex
        bfs_deq = deque()

        bfs_deq.appendleft(v_start)
        while len(bfs_deq) > 0:
            sort_list = []
            vert = bfs_deq.pop()
            vert_len = len(self.adj_list[vert])

            if vert not in reachable:
                reachable.append(vert)

                if vert == v_end and end is True:
                    return reachable

                for index in range(vert_len):
                    sort_list.append(self.adj_list[vert][index])
                sort_list.sort()

                for j in sort_list:
                    bfs_deq.appendleft(j)

        return reachable

    def count_connected_components(self) -> int:
        """
        A method for counting the connected components in the graph. Returns the number of connected components in the
        graph as an integer.
        """

        count = 0
        vert_que = deque()

        for vert in self.adj_list:
            vert_que.append(vert)

        src = vert_que[0]
        # starts at the first vertex in the graph and calls self.dfs to find the reached vertices.
        while len(vert_que) != 0:
            reach = self.dfs(src)

            # removes all visited vertices from the queue
            for vert in reach:
                if vert in vert_que:
                    vert_que.remove(vert)

            count += 1
            # checks if all vertices have been reached, if not the while loop continues with the vertices that haven't
            # been reached.
            if len(vert_que) != 0:
                src = vert_que[0]

        return count

    def has_cycle(self):
        """
        A method for determining if the graph has at least one cycle. Returns True if the graph has a cycle, or false
        graph is acyclic.
        Cite: Open Data Structures (in pseudocode) pg. 252. Implemented the 'parent' approach that was talked about for
        finding cycles using DFS.
        """

        keys = self.get_vertices()
        length = len(keys)

        # makes sure each vertex is checked
        for i in range(length-1):
            visited = {}

            # keeps a dictionary with the vertices of the graph as keys and the value as False until visited.
            for key in keys:
                visited[key] = False

            parent = keys[i] # holds parent vertex
            key = keys[i+1] # starts from the vertex after parent
            visited[key] = True
            dfs_deq = deque()
            dfs_deq.append((key, parent))

            while len(dfs_deq) > 0:

                sort_list = []
                vert, parent = dfs_deq.pop()
                vert_len = len(self.adj_list[vert])

                # iterates through each value of the current vertex key:
                for index in range(vert_len):
                    sort_list.append(self.adj_list[vert][index])

                # checks if each successors' have been visited, appends the child as the new vertex, and current vertex
                # as the parent.
                for j in sort_list:
                    if visited[j] is False:
                        visited[j] = True
                        dfs_deq.append((j, vert))

                    # If successors have been visited, checks whether they are adjacent to the parent.
                    elif j != parent:
                        return True

        return False
