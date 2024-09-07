import numpy as np
from collections import deque


def Reconstruct_path(visited, end):
    path = []
    current_node = end
    
    while current_node is not None:
        path.append(current_node)
        current_node = visited[current_node]  # Move to the previous node in the path
        
    path.reverse()  # Reverse the path to get it from start to end
    return path if path[0] is not None else []


# ========= BFS =========
def BFS(matrix, start, end):
    # Check if the start or end indices are out of bounds
    if start < 0 or end >= len(matrix) or start >= len(matrix):
        raise ValueError("Invalid Input")
    
    visited = {start: None} # Dictionary to keep track of visited nodes and their parents
    frontier = deque([start]) # Queue to manage the nodes to be explored
        
    while frontier:
        current = frontier.popleft()
        
        if current == end:
            break
        
        for vertex, cost in enumerate(matrix[current]):
            if cost > 0 and vertex not in visited: # Check for valid and unvisited neighbors
                visited[vertex] = current # Mark the neighbor as visited with the current node as its parent
                frontier.append(vertex)
                
                if vertex == end:
                    frontier.clear()
                    break
                
    if end not in visited:
        raise ValueError("No path found from start to end")
                
    # Reconstruct the path from start to end
    path = Reconstruct_path(visited, end)
            
    return visited, path


# ========= DFS =========
def dfs_recursion(matrix, visited, current, end):
    if current == end:
        return visited

    for vertex, cost in enumerate(matrix[current]):
        if cost > 0 and vertex not in visited: # Check for valid and unvisited neighbors
            visited[vertex] = current # Mark the neighbor as visited with the current node as its parent
            result = dfs_recursion(matrix, visited, vertex, end) # Recursively explore the neighbor
            if end in result:
                return result

    return visited


def DFS(matrix, start, end):
    # Check if the start or end indices are out of bounds
    if start < 0 or end >= len(matrix) or start >= len(matrix):
        raise ValueError("Invalid Input")
    
    visited = {start: None} # Dictionary to keep track of visited nodes and their parents
    visited = dfs_recursion(matrix, visited, start, end)
    
    if end not in visited:
        raise ValueError("No path found from start to end")
    
    # Reconstruct the path from start to end
    path = Reconstruct_path(visited, end)
    
    return visited, path


# ========= UCS =========
def UCS(matrix, start, end):
    # Check if the start or end indices are out of bounds
    if start < 0 or end >= len(matrix):
        raise ValueError("Invalid Input")
    
    visited = {} # Dictionary to keep track of visited nodes and their parents
    frontier = [(0, start, None)] # Priority queue initialized with the start node
    costs = {start: 0} # Dictionary to keep track of the cost to reach each node
    is_found = False
    
    while frontier:
        # Sort the frontier by cost in descending order and pop the node with the smallest cost
        frontier.sort(reverse=True)
        cost, current_node, parent = frontier.pop()
        
        if current_node in visited:
            continue
        
        visited[current_node] = parent
        
        if current_node == end:
            is_found = True
            break
        
        for vertex, current_cost in enumerate(matrix[current_node]):
            if current_cost > 0: # Check for valid neighbors
                new_cost = cost + current_cost # Calculate the new cost to reach the neighbor
                
                if vertex not in costs or new_cost < costs[vertex]:
                    costs[vertex] = new_cost # Update the cost to reach the neighbor
                    frontier.append((new_cost, vertex, current_node))
    
    if not is_found:
        raise ValueError("No path found from start to end")
    
    # Reconstruct the path from start to end
    path = Reconstruct_path(visited, end)
        
    return visited, path


# ========= GBFS =========
def GBFS(matrix, start, end):
    # Check if the start or end indices are out of bounds
    if start < 0 or end >= len(matrix):
        raise ValueError("Invalid Input")
    
    visited = {} # Dictionary to keep track of visited nodes and their parents
    frontier = [(matrix[start][end], start, None)] # Priority queue initialized with the start node and its heuristic cost
    is_found = False
    
    while frontier:
        # Sort the frontier by heuristic cost in descending order and pop the node with the smallest heuristic cost
        frontier.sort(reverse=True)
        _, current_node, parent = frontier.pop()
        
        if current_node in visited:
            continue
        
        visited[current_node] = parent
        
        if current_node == end:
            is_found = True
            break
        
        for vertex, current_cost in enumerate(matrix[current_node]):
            if current_cost > 0: # Check for valid neighbors
                h_cost = matrix[vertex][end] # Calculate the heuristic cost for the neighbor
                
                if vertex not in visited:
                    frontier.append((h_cost, vertex, current_node))
    
    if not is_found:
        raise ValueError("No path found from start to end")
    
    # Reconstruct the path from start to end
    path = Reconstruct_path(visited, end)
        
    return visited, path


# ========= Astar =========

# Calculate the heuristic cost for A* by Euclidean distance
def euclidean_distance(node, goal):
    return np.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)


def Astar(matrix, start, end, pos):
    # Check if the start or end indices are out of bounds
    if start < 0 or end >= len(matrix):
        raise ValueError("Invalid Input")
   
    visited = {} # Dictionary to keep track of visited nodes and their parents
    frontier = {start: (0, 0,None)} # Priority queue initialized with the start node
    is_found = False

    while frontier:
        # Pop the node with the smallest cost + heuristic value
        current_node, (cost_node, h, adjacent) = frontier.popitem()
        visited[current_node] = adjacent # Mark the node as visited

        if current_node == end:
            is_found = True
            break

        for vertex, cost in  enumerate(matrix[current_node]):
            if vertex not in visited and cost > 0: # Check for valid and unvisited neighbors
                new_cost = cost_node + cost # Calculate the new cost to reach the neighbor
                
                if vertex in frontier:
                    old_cost = frontier[vertex][0]
                    if old_cost > (cost_node + cost): # Update the cost if a cheaper path is found
                        frontier[vertex] = (new_cost, frontier[vertex][1], current_node)
                else:
                    # Calculate the heuristic cost using Euclidean distance
                    vertex_pos = (vertex, vertex)
                    end_pos = (end, end)
                    frontier[vertex] = (new_cost, euclidean_distance(vertex_pos, end_pos), current_node)

        # Sort the frontier by the sum of cost + heuristic value
        frontier = dict(sorted(frontier.items(), key=lambda item: (item[1][0] + item[1][1], item[0]), reverse=True))

    if not is_found:
        raise ValueError("No path found from start to end")
    
    # Reconstruct the path from start to end
    path = Reconstruct_path(visited, end)
    
    return visited, path
