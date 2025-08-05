
import matplotlib.pyplot as plt
import heapq
import math
import random
import time

class Node:
    def __init__(self, position, g=0, h=0, parent=None):
        self.position = position
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def get_neighbors(node, grid):
    directions = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
    neighbors = []
    for dx, dy in directions:
        x, y = node.position[0] + dx, node.position[1] + dy
        if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 0:
            neighbors.append((x, y))
    return neighbors

def reconstruct_path(node):
    path = []
    while node:
        path.append(node.position)
        node = node.parent
    return path[::-1]

def a_star(grid, start, goal, verbose=False, visualize=False):
    open_set = []
    start_node = Node(start, g=0, h=heuristic(start, goal))
    heapq.heappush(open_set, start_node)
    visited = set()
    explored_nodes = 0
    visited_nodes = []

    t0 = time.time()
    while open_set:
        current = heapq.heappop(open_set)
        explored_nodes += 1
        visited.add(current.position)
        if visualize:
            visited_nodes.append(current.position)

        if current.position == goal:
            t1 = time.time()
            if verbose:
                print(f"\n탐색 완료: {start} → {goal} | 노드 탐색 수: {explored_nodes} | 소요 시간: {t1 - t0:.4f}초")
            return reconstruct_path(current), visited_nodes

        for pos in get_neighbors(current, grid):
            if pos in visited:
                continue
            g = current.g + heuristic(current.position, pos)
            h = heuristic(pos, goal)
            neighbor = Node(pos, g, h, current)
            heapq.heappush(open_set, neighbor)

    t1 = time.time()
    if verbose:
        print(f"\n경로 없음: {start} → {goal} | 노드 탐색 수: {explored_nodes} | 소요 시간: {t1 - t0:.4f}초")
    return None, visited_nodes

def find_path_with_waypoints(grid, start, waypoints, goal, verbose=False, visualize=False):
    full_path = []
    visited_all = []
    current = start

    for i, wp in enumerate(waypoints):
        print(f"\n🔄 {i+1}) {current} → {wp}")
        segment, visited = a_star(grid, current, wp, verbose=verbose, visualize=visualize)
        if segment is None:
            print(f"경유지 {wp} 까지 경로 없음!")
            return None, visited_all
        if full_path:
            segment = segment[1:]
        full_path += segment
        visited_all += visited
        current = wp

    print(f"\n마지막: {current} → {goal}")
    segment, visited = a_star(grid, current, goal, verbose=verbose, visualize=visualize)
    if segment is None:
        print("최종 목적지까지 경로 없음!")
        return None, visited_all
    if full_path:
        segment = segment[1:]
    full_path += segment
    visited_all += visited

    print("전체 경로 계산 완료!\n")
    return full_path, visited_all

def visualize_search_with_dynamic_obstacles(grid, path, visited_nodes, start, goal, waypoints=None, dynamic_obstacles=None):
    plt.figure(figsize=(6, 6))
    visited_set = set()
    for i, (x, y) in enumerate(visited_nodes):
        if dynamic_obstacles and i % 15 == 0:
            for dx, dy in dynamic_obstacles:
                if 0 <= dx < len(grid) and 0 <= dy < len(grid[0]):
                    grid[dx][dy] = 1
        plt.plot(y, len(grid) - x - 1, 'c.', alpha=0.3)
        plt.pause(0.001)
        visited_set.add((x, y))

    for x in range(len(grid)):
        for y in range(len(grid[0])):
            if grid[x][y] == 1:
                plt.plot(y, len(grid) - x - 1, 'ks')
            elif (x, y) not in visited_set:
                plt.plot(y, len(grid) - x - 1, 'ws')

    if path:
        for (x, y) in path:
            plt.plot(y, len(grid) - x - 1, 'go')

    if start:
        plt.plot(start[1], len(grid) - start[0] - 1, 'bo', label="Start")
    if goal:
        plt.plot(goal[1], len(grid) - goal[0] - 1, 'ro', label="Goal")
    if waypoints:
        for i, (x, y) in enumerate(waypoints):
            plt.plot(y, len(grid) - x - 1, 'yo')
            plt.text(y + 0.2, len(grid) - x - 1 + 0.2, f'W{i+1}', fontsize=8, color='orange')

    plt.grid(True)
    plt.legend()
    plt.title("A* Pathfinding with Waypoints")
    plt.show()

def draw(grid, path=None, start=None, goal=None, waypoints=None, visited_nodes=None):
    plt.figure(figsize=(6, 6))
    for x in range(len(grid)):
        for y in range(len(grid[0])):
            if grid[x][y] == 1:
                plt.plot(y, len(grid) - x - 1, 'ks')
            else:
                plt.plot(y, len(grid) - x - 1, 'ws')

    if visited_nodes:
        for (x, y) in visited_nodes:
            plt.plot(y, len(grid) - x - 1, 'c.', alpha=0.3)

    if path:
        for (x, y) in path:
            plt.plot(y, len(grid) - x - 1, 'go')

    if start:
        plt.plot(start[1], len(grid) - start[0] - 1, 'bo', label="Start")
    if goal:
        plt.plot(goal[1], len(grid) - goal[0] - 1, 'ro', label="Goal")
    if waypoints:
        for i, (x, y) in enumerate(waypoints):
            plt.plot(y, len(grid) - x - 1, 'yo')
            plt.text(y + 0.2, len(grid) - x - 1 + 0.2, f'W{i+1}', fontsize=8, color='orange')

    plt.grid(True)
    plt.legend()
    plt.title("A* Final Path Summary")
    plt.show()

def generate_random_obstacles(grid, count):
    size = len(grid)
    for _ in range(count):
        x, y = random.randint(0, size-1), random.randint(0, size-1)
        grid[x][y] = 1

if __name__ == "__main__":
    size = 20
    grid = [[0 for _ in range(size)] for _ in range(size)]
    generate_random_obstacles(grid, 50)
    dynamic_obstacles = [(7, 7), (14, 14), (16, 8)]

    start = (0, 0)
    waypoints = [(4, 6), (2, 12), (11, 12)]
    goal = (19, 19)
    

    path, visited_nodes = find_path_with_waypoints(grid, start, waypoints, goal, verbose=True, visualize=True)
    visualize_search_with_dynamic_obstacles(grid, path, visited_nodes, start, goal, waypoints, dynamic_obstacles)
    draw(grid, path, start, goal, waypoints, visited_nodes, dynamic_obstacles)
