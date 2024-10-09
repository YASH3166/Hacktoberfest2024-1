import heapq

def dijkstra(graph, start):
    # Initialize the priority queue (min-heap)
    pq = []
    heapq.heappush(pq, (0, start))  # (distance, node)

    # Distances dictionary with initial values as infinity
    distances = {node: float('infinity') for node in graph}
    distances[start] = 0  # Distance to the start node is 0

    # Previous nodes to track the shortest path (optional)
    previous_nodes = {node: None for node in graph}

    while pq:
        # Pop the node with the smallest distance
        current_distance, current_node = heapq.heappop(pq)

        # If the popped node's distance is greater than the stored distance, continue
        if current_distance > distances[current_node]:
            continue

        # Explore neighbors
        for neighbor, weight in graph[current_node]:
            distance = current_distance + weight

            # If a shorter path to the neighbor is found
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(pq, (distance, neighbor))

    return distances, previous_nodes

# Example graph representation
graph = {
    'A': [('B', 1), ('C', 4)],
    'B': [('A', 1), ('C', 2), ('D', 5)],
    'C': [('A', 4), ('B', 2), ('D', 1)],
    'D': [('B', 5), ('C', 1)]
}

# Running the Dijkstra algorithm from node 'A'
distances, previous_nodes = dijkstra(graph, 'A')

# Output the shortest distances from node 'A'
print("Shortest distances from A:", distances)

# Output the shortest path to each node
print("Previous nodes:", previous_nodes)
