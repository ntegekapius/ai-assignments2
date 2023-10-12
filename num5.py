
from queue import PriorityQueue
custom_graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'B': 2, 'C': 2},
    'B': {'C': 3},
    'C': {'D': 4, 'G': 4},
    'D': {'G': 1},
    'G': {}
}


custom_heuristics = {
    'S': 7,
    'A': 5,
    'B': 7,
    'C': 4,
    'D': 1,
    'G': 0
}



def custom_depth_first_search(graph, start, target):
    custom_stack = [(start, [start])]
    visited_nodes = set()  # To keep track of visited nodes
    expanded_states = []  # To store the order of expanded states
    while custom_stack:
        (node, path) = custom_stack.pop()
        visited_nodes.add(node)  # Mark the current node as visited
        expanded_states.append(node)  # Add the expanded state to the list
        if node == target:
            return expanded_states, path
        neighbors = graph[node]
        for neighbor in sorted(neighbors, reverse=True):  # Break ties alphabetically by reversing the order
            if neighbor not in path and neighbor not in visited_nodes:
                custom_stack.append((neighbor, path + [neighbor]))

def custom_breadth_first_search(graph, start, target):
    custom_queue = [(start, [start])]
    visited_nodes = set()  # To keep track of visited nodes
    expanded_states = []  # To store the order of expanded states
    while custom_queue:
        (node, path) = custom_queue.pop(0)
        visited_nodes.add(node)  # Mark the current node as visited
        expanded_states.append(node)  # Add the expanded state to the list
        if node == target:
            return expanded_states, path
        neighbors = graph[node]
        for neighbor in sorted(neighbors):  # Break ties alphabetically
            if neighbor not in path and neighbor not in visited_nodes:
                custom_queue.append((neighbor, path + [neighbor]))

def custom_uniform_cost_search(graph, start, target):
    custom_priority_queue = [(0, start, [start])]
    visited_nodes = set()  # To keep track of visited nodes
    expanded_states = []  # To store the order of expanded states
    while custom_priority_queue:
        (cost, node, path) = custom_priority_queue.pop(0)
        visited_nodes.add(node)  # Mark the current node as visited
        expanded_states.append(node)  # Add the expanded state to the list
        if node == target:
            return expanded_states, path
        neighbors = graph[node]
        for neighbor, neighbor_cost in sorted(neighbors.items()):  # Break ties alphabetically
            if neighbor not in path and neighbor not in visited_nodes:
                new_cost = cost + neighbor_cost
                custom_priority_queue.append((new_cost, neighbor, path + [neighbor]))
                custom_priority_queue.sort(key=lambda x: (x[0], x[1]))  # Sort by cost and then by name

def custom_greedy_search(graph, start, target, heuristics):
    custom_priority_queue = [(heuristics[start], start, [start])]
    visited_nodes = set()  # To keep track of visited nodes
    expanded_states = []  # To store the order of expanded states
    while custom_priority_queue:
        (_, node, path) = custom_priority_queue.pop(0)
        visited_nodes.add(node)  # Mark the current node as visited
        expanded_states.append(node)  # Add the expanded state to the list
        if node == target:
            return expanded_states, path
        neighbors = graph[node]
        for neighbor in sorted(neighbors, key=lambda x: heuristics[x]):  # Break ties alphabetically based on heuristic value
            if neighbor not in path and neighbor not in visited_nodes:
                custom_priority_queue.append((heuristics[neighbor], neighbor, path + [neighbor]))
                custom_priority_queue.sort(key=lambda x: (x[0], x[1]))  # Sort by heuristic and then by name

def custom_a_star_search(graph, start, target, heuristics):
    custom_priority_queue = [(heuristics[start], 0, start, [start])]
    visited_nodes = set()  # To keep track of visited nodes
    expanded_states = []  # To store the order of expanded states
    while custom_priority_queue:
        (_, cost, node, path) = custom_priority_queue.pop(0)
        visited_nodes.add(node)  # Mark the current node as visited
        expanded_states.append(node)  # Add the expanded state to the list
        if node == target:
            return expanded_states, path
        neighbors = graph[node]
        for neighbor, neighbor_cost in sorted(neighbors.items(), key=lambda x: heuristics[x[0]]):  # Break ties alphabetically based on heuristic value
            if neighbor not in path and neighbor not in visited_nodes:
                new_cost = cost + neighbor_cost
                custom_priority_queue.append((heuristics[neighbor], new_cost, neighbor, path + [neighbor]))
                custom_priority_queue.sort(key=lambda x: (x[0], x[1]))  # Sort by heuristic and then by name

# Test the custom search algorithms
custom_start_node = 'S'
custom_target_node = 'G'

# Depth-First Search
expanded_states_dfs, dfs_path = custom_depth_first_search(custom_graph, custom_start_node, custom_target_node)
print("DFS Expanded States:", expanded_states_dfs)
print("DFS Path:", dfs_path)
print("DFS States Not Expanded:", [node for node in custom_graph if node not in expanded_states_dfs])

# Breadth-First Search
expanded_states_bfs, bfs_path = custom_breadth_first_search(custom_graph, custom_start_node, custom_target_node)
print("\nBFS Expanded States:", expanded_states_bfs)
print("BFS Path:", bfs_path)
print("BFS States Not Expanded:", [node for node in custom_graph if node not in expanded_states_bfs])

# Uniform Cost Search
expanded_states_ucs, ucs_path = custom_uniform_cost_search(custom_graph, custom_start_node, custom_target_node)
print("\nUCS Expanded States:", expanded_states_ucs)
print("UCS Path:", ucs_path)
print("UCS States Not Expanded:", [node for node in custom_graph if node not in expanded_states_ucs])

# Greedy Search
expanded_states_greedy, greedy_path = custom_greedy_search(custom_graph, custom_start_node, custom_target_node, custom_heuristics)
print("\nGreedy Search Expanded States:", expanded_states_greedy)
print("Greedy Search Path:", greedy_path)
print("Greedy Search States Not Expanded:", [node for node in custom_graph if node not in expanded_states_greedy])

# A* Search
expanded_states_a_star, a_star_path = custom_a_star_search(custom_graph, custom_start_node, custom_target_node, custom_heuristics)
print("\nA* Search Expanded States:", expanded_states_a_star)
print("A* Search Path:", a_star_path)
print("A* Search States Not Expanded:", [node for node in custom_graph if node not in expanded_states_a_star])
