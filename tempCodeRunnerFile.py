import numpy as np

MOVES = {
    'U': (-1, 0),
    'D': (1, 0),
    'L': (0, -1),
    'R': (0, 1)
}

class PuzzleNode:
    def __init__(self, state, parent=None, move=None, cost=0, heuristic=0):
        self.state = state
        self.parent = parent
        self.move = move
        self.cost = cost
        self.heuristic = heuristic

    def __lt__(self, other):
        return self.cost + self.heuristic < other.cost + other.heuristic

def manhattan_distance(state, goal):
    distance = 0
    for value in range(1, len(state) * len(state[0])):
        x1, y1 = np.where(state == value)
        x2, y2 = np.where(goal == value)
        distance += abs(x1 - x2) + abs(y1 - y2)
    return distance

def generate_neighbors(node, goal):
    neighbors = []
    state = node.state
    blank_pos = np.where(state == 16)
    x, y = blank_pos[0][0], blank_pos[1][0]

    for move, (dx, dy) in MOVES.items():
        new_x, new_y = x + dx, y + dy
        if 0 <= new_x < state.shape[0] and 0 <= new_y < state.shape[1]:
            new_state = state.copy()
            new_state[x, y], new_state[new_x, new_y] = new_state[new_x, new_y], new_state[x, y]
            new_cost = node.cost + 1
            new_heuristic = manhattan_distance(new_state, goal)
            neighbors.append(PuzzleNode(new_state, node, move, new_cost, new_heuristic))
    return neighbors
def count_inversions(state):
    # Flatten the 4x4 state matrix while ignoring the blank (represented by 16)
    flattened_state = state.flatten()
    inversions = 0
    
    # Count the number of inversions
    for i in range(len(flattened_state)):
        for j in range(i + 1, len(flattened_state)):
            if flattened_state[i] != 16 and flattened_state[j] != 16 and flattened_state[i] > flattened_state[j]:
                inversions += 1
    
    return inversions

def is_solvable(state):
    # Calculate the number of inversions
    inversions = count_inversions(state)
    
    # Find the row index of the blank space (value 16)
    blank_pos = np.where(state == 16)
    blank_row_from_bottom = 4 - blank_pos[0][0]  # As grid is always 4x4, row count is 4

    # Solvability rule for 4x4 grid:
    # If the blank is on an even row counting from the bottom (1-based), 
    # then the puzzle is solvable if the number of inversions is odd.
    # If the blank is on an odd row counting from the bottom, 
    # then the puzzle is solvable if the number of inversions is even.
    if blank_row_from_bottom % 2 == 0:
        return inversions % 2 == 1
    else:
        return inversions % 2 == 0
def ida_star(start_state, goal_state):
    threshold = manhattan_distance(start_state, goal_state)

    def search(path, g, threshold):
        node = path[-1]
        f = g + node.heuristic
        if f > threshold:
            return f
        if np.array_equal(node.state, goal_state):
            return path
        min_threshold = float('inf')
        for neighbor in generate_neighbors(node, goal_state):
            if not path or not np.array_equal(neighbor.state, path[-1].state):
                path.append(neighbor)
                result = search(path, g + 1, threshold)
                if isinstance(result, list):
                    return result
                if result < min_threshold:
                    min_threshold = result
                path.pop()
        return min_threshold

    start_node = PuzzleNode(start_state, heuristic=manhattan_distance(start_state, goal_state))
    path = [start_node]
    while True:
        result = search(path, 0, threshold)
        if isinstance(result, list):
            return result
        if result == float('inf'):
            return None
        threshold = result

if is_solvable(initial_state):
    solution_path = ida_star(initial_state, goal_state)

    if solution_path:
        for node in solution_path:
            print(node.state, "\n")
    else:
        print("Aucune solution trouvée")
else:
    print("Aucune solution trouvée")