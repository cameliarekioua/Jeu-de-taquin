import numpy as np

MOVES = {
    'U': (-1, 0),  # Up
    'D': (1, 0),   # Down
    'L': (0, -1),  # Left
    'R': (0, 1)    # Right
}

class PuzzleNode:
    def __init__(self, state, parent=None, move=None, cost=0, heuristic=0):
        self.state = state
        self.parent = parent
        self.move = move
        self.cost = cost
        self.heuristic = heuristic

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

def manhattan_distance(state, goal):
    """
    Calculate the Manhattan distance between the current state and the goal state.
    """
    distance = 0
    rows, cols = state.shape
    for value in range(1, rows * cols):
        x1, y1 = np.where(state == value)
        x2, y2 = np.where(goal == value)
        distance += abs(x1 - x2) + abs(y1 - y2)
    return distance

def generate_neighbors_and_heuristics(node, goal):
    """
    Generate neighbors of the current node and calculate their cost and heuristic.
    """
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
    """
    Count the number of inversions in the current state.
    """
    flattened_state = state.flatten()
    inversions = 0

    for i in range(len(flattened_state)):
        for j in range(i + 1, len(flattened_state)):
            if flattened_state[i] != 16 and flattened_state[j] != 16 and flattened_state[i] > flattened_state[j]:
                inversions += 1

    return inversions

def is_solvable(state):
    """
    Check if the current state of the puzzle is solvable based on the number of inversions
    and the position of the blank space.
    """
    inversions = count_inversions(state)
    blank_pos = np.where(state == 16)
    blank_row_from_bottom = 4 - blank_pos[0][0]  # Number of rows below the blank space

    # Return True if (inversions + blank_row_from_bottom) is odd
    return (inversions + blank_row_from_bottom) % 2 == 1

def solve_puzzle(start_state, goal_state):
    """
    Solve the 4x4 sliding puzzle using the IDA* algorithm.
    """
    if not is_solvable(start_state):
        print("Puzzle is unsolvable.")
        return

    def ida_star_search(path, g, threshold):
        node = path[-1]
        f = g + node.heuristic
        if f > threshold:
            return f
        if np.array_equal(node.state, goal_state):
            return path
        min_threshold = float('inf')
        for neighbor in generate_neighbors_and_heuristics(node, goal_state):
            if not path or not np.array_equal(neighbor.state, path[-1].state):
                path.append(neighbor)
                result = ida_star_search(path, g + 1, threshold)
                if isinstance(result, list):
                    return result
                if result < min_threshold:
                    min_threshold = result
                path.pop()
        return min_threshold

    threshold = manhattan_distance(start_state, goal_state)
    start_node = PuzzleNode(start_state, heuristic=threshold)
    path = [start_node]

    while True:
        result = ida_star_search(path, 0, threshold)
        if isinstance(result, list):
            for node in result:
                print(node.state, "\n")
            return
        if result == float('inf'):
            print("No solution found.")
            return
        threshold = result

# Example usage
initial_state = np.array([[1, 2, 3, 4],
                          [5, 6, 8, 16],
                          [9, 10, 7, 11],
                          [13, 14, 15, 12]])

goal_state = np.array([[1, 2, 3, 4],
                       [5, 6, 7, 8],
                       [9, 10, 11, 12],
                       [13, 14, 15, 16]])

solve_puzzle(initial_state, goal_state)
