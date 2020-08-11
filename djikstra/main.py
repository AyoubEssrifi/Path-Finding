from math import inf

graph = {'A': {'B': 6, 'D':1},
        'B': {'A': 6, 'E': 2, 'C':5},
        'D': {'A': 1, 'B': 2, 'E':1},
        'E': {'B': 2, 'C': 5, 'D':1},
        'C': {'B': 5, 'E':5}
        }

start = 'A'
finish = 'C'

def print_graph(graph):
    """
    Prints the graph 

    Args:
        graph (dict): dictionary with nodes as keys and values are a dictionary which keys are are adjacent nodes and values are the distance
    """
    for node in graph.keys():
        for neighbour in graph[node].keys():
            print("{0} -> {1}, weight: {2}".format(node, neighbour, graph[node][neighbour]))

def get_min_dist_node(dist, visited):
    """
    Gets the node with the minimum distance from the unvisited nodes

    Args:
        dist (dict): dictionary with nodes as keys and values is a liste which first element is the minimum distance and second one is the closest node
        visited (array): array of visited nodes

    Returns:
        (string): node with minimum distance
    """
    a = []
    for key in dist.keys(): 
        if key not in visited:
            a.append((dist[key][0], key))
    idx = a.index(min(a))
    return a[idx][1]

def djikstra(start):
    """
    Performs the djisktra's algorithm

    Args:
        start (string): starting node
    
    Returns:
        shortest_dist (dict): dictionary with nodes as keys and values is a liste which first element is the minimum distance and second one is the closest node
    """
    # Initialization
    shortest_dist = {}
    for node in graph.keys():
        shortest_dist[node] = [inf, None]
    shortest_dist[start][0] = 0
    visited = []
    unvisited = list(graph.keys())

    # Djikstra's Algorithm
    i = 0
    while i < len(unvisited):
        current_node = get_min_dist_node(shortest_dist, visited)
        for neighbour in graph[current_node].keys():
            d = graph[current_node][neighbour] + shortest_dist[current_node][0]
            if d < shortest_dist[neighbour][0]:
                shortest_dist[neighbour][0] = d
                shortest_dist[neighbour][1] = current_node
        visited.append(current_node)
        i += 1

    return shortest_dist

def find_path(shortest_dist, next_node, path=[]):
    """
    Find recursively the path propagating backwards from finish to start

    Args:
        shortest_dist (dict): 
        next_node (string): next node
        path (list, optional): list of nodes that form the path. Defaults to [].

    Returns:
        path (array): array of nodes that form the shortest path
    """
    path.insert(0, next_node)
    if next_node == start:
        return path
    next_node = shortest_dist[next_node][1]
    return find_path(shortest_dist, next_node, path)

def find_shortest_path(graph, start, finish):
    """
    Finds the shortest path of graph from a start node to a finish node

    Args:
        graph (dict): dictionary with nodes as keys and values is a liste which first element is the minimum distance and second one is the closest node
        start (string): starting node
        finish (string): finishing node
    """
    shortest_dist = djikstra(start)
    ans = find_path(shortest_dist, finish)
    print(" -> ".join(ans))

if __name__ == '__main__':
    print_graph(graph)
    find_shortest_path(graph, start, finish)



 
       

  

