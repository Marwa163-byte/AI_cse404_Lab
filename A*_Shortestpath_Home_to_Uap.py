import heapq

# Define the graph with distances between locations
graph = {
    'Home': {'Shymoli': 2.6, 'Mohammadpur': 2.0, 'CollegeGate': 3.0},
    'Shymoli': {'Khamarbari': 3.2},
    'Khamarbari': {'Farmgate': 0.8},
    'Farmgate': {'UAP': 0.8},  # Farmgate connects to UAP
    'UAP': {'Farmgate': 0.8},  # UAP is reached from Farmgate
    'Mohammadpur': {'Asadgate': 2.0, 'Jigatola': 3.1, 'Panthapath': 4.8},
    'Asadgate': {'Farmgate': 2.2},
    'Jigatola': {'UAP': 4.3},
    'Panthapath': {'UAP': 0.6},
    'CollegeGate': {'Farmgate': 3.7, 'Dhanmondi': 4.5, 'Kalabagan': 4.2},
    'Dhanmondi': {'UAP': 4.0},
    'Kalabagan': {'UAP': 2.5},
}

# Heuristic values from the table (estimated distance to UAP)
heuristics = {
    'Home': 0.040,
    'Shymoli': 0.032,
    'Khamarbari': 0.005,
    'Farmgate': 0.004,
    'Mohammadpur': 0.029,
    'Jigatola': 0.024,
    'Panthapath': 0.006,
    'Dhanmondi': 0.023,
    'Kalabagan': 0.013,
    'CollegeGate': 0.027,
    'UAP': 0.0,
    'Asadgate': 0.019  
}

# A* algorithm to find the shortest path
def a_star(start, goal):
    open_list = []
    closed_list = set()

    g_score = {start: 0}
    f_score = {start: heuristics[start]}

    heapq.heappush(open_list, (f_score[start], start))

    came_from = {}

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        closed_list.add(current)

        for neighbor, distance in graph[current].items():
            if neighbor in closed_list:
                continue

            tentative_g_score = g_score[current] + distance

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristics[neighbor]
                came_from[neighbor] = current
                heapq.heappush(open_list, (f_score[neighbor], neighbor))

    return None

# Run the A* algorithm
start_location = 'Home'
goal_location = 'UAP'
path = a_star(start_location, goal_location)

print("Shortest path:", path)
