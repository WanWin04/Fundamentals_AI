import numpy as np
from collections import deque
from queue import PriorityQueue
import math


def path_converter(visited, end):
    path = []
    current_node = end
    
    while current_node is not None:
        path.append(current_node)
        current_node = visited[current_node]
        
    path.reverse()
    return path if path[0] is not None else []


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
        raise ValueError("Invalid Input")
    
    visited = {start: None}
    queue = deque([start])
        
    while queue:
        current = queue.popleft()
        
        if current == end:
            break
        
        for vertex, cost in enumerate(matrix[current]):
            if cost > 0 and vertex not in visited:
                visited[vertex] = current
                queue.append(vertex)
                if vertex == end:
                    queue.clear()
                    break
                
    path = path_converter(visited, end)
        
    return visited, path


def dfs_recursion(matrix, visited, current, end):
    if current == end:
        return visited

    for vertex, cost in enumerate(matrix[current]):
        if cost > 0 and vertex not in visited:
            visited[vertex] = current
            result = dfs_recursion(matrix, visited, vertex, end)
            if end in result:
                return result

    return visited


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
        raise ValueError("Invalid Input")
    
    visited = {start: None}
    visited = dfs_recursion(matrix, visited, start, end)
    
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
        raise ValueError("Invalid Input")
    
    visited = {start: None}
    cost_so_far = {start: 0}
    queue = PriorityQueue()
    queue.put((0, start))
    
    while not queue.empty():
        current_cost, current_node = queue.get()
        
        if current_node == end:
            break
        
        for vertex, cost in enumerate(matrix[current_node]):
            if cost > 0:
                new_cost = current_cost + cost
                if vertex not in cost_so_far or new_cost < cost_so_far[vertex]:
                    cost_so_far[vertex] = new_cost
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
        raise ValueError("Invalid Input")
    
    visited = {start: None}
    queue = PriorityQueue()
    queue.put((0, start))
    
    while not queue.empty():
        current = queue.get()[1]
        
        if current == end:
            break
        
        for vertex, cost in enumerate(matrix[current]):
            if cost > 0 and vertex not in visited:
                visited[vertex] = current
                queue.put((cost, vertex))
                
    path = path_converter(visited, end)
    
    return visited, path


def calculate_g_cost(matrix, visited, vertex):
    cost = 0
    current_node = vertex
    while current_node is not None:
        parent_node = visited[current_node]
        if parent_node is not None:
            cost += matrix[parent_node][current_node]
        current_node = parent_node
    return cost


def euclidean_distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)


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
    
    if start < 0 or end >= len(matrix):
        raise ValueError("Invalid Input")
    
    visited = {start: None}
    open_list = PriorityQueue()
    open_list.put((0, start))
    g_cost = {start: 0}
    closed_list = set()
    
    while not open_list.empty():
        current = open_list.get()[1]
        
        if current == end:
            break
        
        closed_list.add(current)
        
        for vertex, cost in enumerate(matrix[current]):
            if cost > 0 and vertex not in closed_list:
                tentative_g_cost = g_cost[current] + cost
                
                if vertex not in g_cost or tentative_g_cost < g_cost[vertex]:
                    g_cost[vertex] = tentative_g_cost
                    f = tentative_g_cost + euclidean_distance(pos[vertex], pos[end]) # f = g + h
                    open_list.put((f, vertex))
                    visited[vertex] = current
                
    path = path_converter(visited, end)
    
    return visited, path
