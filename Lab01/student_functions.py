import numpy as np
from collections import deque
from queue import PriorityQueue


def path_converter(visited, end):
    path = []
    
    if end not in visited:
        print("Not Found Path")
        return path
    
    previous_node = visited[end]
    path.append(end)
    
    while previous_node in visited and previous_node is not None:
        path.append(previous_node)
        previous_node = visited[previous_node]
        
    path.reverse()
    return path


def DFS_recur(matrix, visited, current, end):
    if current == end:
        return visited
    
    for vertex in range(len(matrix)):
        cost = matrix[current][vertex]
        if cost > 0 and vertex not in visited:
            visited[vertex] = current
            visited = DFS_recur(matrix, visited, vertex, end)
            
    return visited


def current_cost_of_vertex(visited, vertex, matrix):
    path_cost = 0
    
    while visited[vertex] is not None:
        path_cost += matrix[vertex][visited[vertex]]
        vertex = visited[vertex]
        
    return path_cost


def BFS(matrix, start, end):
    """
    BFS algorithm:
    Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes, each key is a visited node,
        each value is the adjacent node visited before it.
    path: list
        Founded path
    """
    
    if start < 0 or end >= len(matrix):
        print("Invalid Input")
        return {}, []
    
    visited = {start: None}
    queue = deque()
    queue.append(start)
    
    while True:
        current = queue.popleft()
        if current == end:
            break
        
        for vertex in range(len(matrix)):
            cost = matrix[current][vertex]
            if cost > 0 and vertex not in visited:
                visited[vertex] = current
                queue.append(vertex)
                if vertex == end:
                    break
                
    path = path_converter(visited, end)
    
    return visited, path


def DFS(matrix, start, end):
    """
    DFS algorithm
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited 
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """

    if start < 0 or end >= len(matrix):
        print("Invalid Input")
        return {}, []
    
    visited = {start: None}
    visited = DFS_recur(matrix, visited, start, end)
    path = path_converter(visited, end)
    
    return visited, path


def UCS(matrix, start, end):
    """
    Uniform Cost Search algorithm
     Parameters:visited
    ---------------------------
    matrix: np array
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    
    if start < 0 or end >= len(matrix):
        print("Invalid Input")
        return {}, []
    
    visited = {start: None}
    queue = PriorityQueue()
    queue.put((0, start))
    
    while not queue.empty():
        current_cost, current_node = queue.get()
        
        if current_node == end:
            break
        
        for vertex in range(len(matrix)):
            cost = matrix[current_node][vertex]
            
            if cost <= 0 or vertex in visited:
                continue
            
            new_cost = current_cost + cost
            
            if vertex not in visited or new_cost < current_cost_of_vertex(visited, vertex, matrix):
                visited[vertex] = current_node
                queue.put((new_cost, vertex))
    
    path = path_converter(visited, end)
    
    return visited, path


def GBFS(matrix, start, end):
    """
    Greedy Best First Search algorithm 
    heuristic : edge weights
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
   
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
     
    if start < 0 or end >= len(matrix):
        print("Invalid Input")
        return {}, []
    
    visited = {start: None}
    queue = PriorityQueue()
    queue.put((0, start))
    
    while queue.not_empty:
        current = queue.get()[1]
        
        if current == end:
            break
        
        for vertex in range(len(matrix)):
            cost = matrix[current][vertex]
            if cost > 0 and vertex not in visited:
                visited[vertex] = current
                queue.put((cost, vertex))
                
    path = path_converter(visited, end)
    
    return visited, path


def Astar(matrix, start, end, pos):
    """
    A* Search algorithm
    heuristic: eclid distance based positions parameter
     Parameters:
    ---------------------------
    matrix: np array UCS
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    pos: dictionary. keys are nodes, values are positions
        positions of graph nodes
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO: 
