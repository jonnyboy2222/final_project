import matplotlib.pyplot as plt
import heapq
import math
import random
import time
from tqdm import tqdm

class Node:
    def __init__(self, position, g=0, h=0, parent=None):
        self.position = position # 현재 위치 (x, y)
        self.g = g # 시작점에서 현재까지의 실제거리
        self.h = h # 현재 위치에서 목표까지의 추정 거리
        self.f = g + h # 총 비용 (우선순위 큐 기준)
        self.parent = parent # 경로 추적용 부모 노드
    def __lt__(self, other):
        return self.f < other.f # heapq에서 작은 f값이 먼저 나오게 함
    
# :흰색_확인_표시: Euclidean distance heuristic
def heuristic(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def get_neighbors(node, grid):
    directions = [  # 8방향 허용
        (-1, -1), (-1, 0), (-1, 1),
        ( 0, -1),          ( 0, 1),
        ( 1, -1), ( 1, 0), ( 1, 1)
    ]
    neighbors = []
    for dx, dy in directions:
        x, y = node.position[0] + dx, node.position[1] + dy
        if 0 <= x < len(grid) and 0 <= y < len(grid[0]):
            if grid[x][y] == 0:
                neighbors.append((x, y))
    return neighbors

def reconstruct_path(node):
    path = []
    while node:
        path.append(node.position)
        node = node.parent
    return path[::-1]

def a_star(grid, start, goal, verbose=False):
    open_set = []
    start_node = Node(start, g=0, h=heuristic(start, goal))
    heapq.heappush(open_set, start_node)
    visited = set()
    explored_nodes = 0
    t0 = time.time()

    while open_set:
        current = heapq.heappop(open_set)
        explored_nodes += 1
        if current.position == goal:
            t1 = time.time()
            if verbose:
                print(f"탐색 완료: {start} → {goal} | 노드 탐색 수: {explored_nodes} | 소요 시간: {t1 - t0:.4f}초")
            return reconstruct_path(current)
        visited.add(current.position)
        for pos in get_neighbors(current, grid):
            if pos in visited:
                continue
            g = current.g + heuristic(current.position, pos)
            h = heuristic(pos, goal)
            neighbor = Node(pos, g, h, current)
            heapq.heappush(open_set, neighbor)

    t1 = time.time()

    if verbose:
        print(f"경로 없음: {start} → {goal} | 노드 탐색 수: {explored_nodes} | 소요 시간: {t1 - t0:.4f}초")
    return None

def find_path_with_waypoints(grid, start, waypoints, goal):
    full_path = []
    current = start
    print("\n경유지를 순서대로 탐색 중...")
    for idx, wp in enumerate(tqdm(waypoints, desc="Finding path via waypoints")):
        print(f"{idx+1}) {current} → {wp}")
        segment = a_star(grid, current, wp, verbose=True)
        if segment is None:
            print(f"경유지 {wp} 까지 경로 없음!")
            return None
        if full_path:
            segment = segment[1:]
        full_path += segment
        current = wp
    print("\n최종 목적지까지 경로 탐색 중...")
    print(f"{current} → {goal}")
    segment = a_star(grid, current, goal, verbose=True)
    if segment is None:
        print("최종 목적지까지 경로 없음!")
        return None
    if full_path:
        segment = segment[1:]
    full_path += segment
    print("전체 경로 계산 완료!\n")
    return full_path

# 장애물 랜덤 생성 + 실시간 재탐색 루프
def generate_random_obstacles(grid, count):
    for _ in range(count):
        x, y = random.randint(0, len(grid)-1), random.randint(0, len(grid[0])-1)
        if grid[x][y] == 0 and (x, y) != start and (x, y) != goal:
            grid[x][y] = 1

def simulate_with_replanning(grid, start, goal):
    path = a_star(grid, start, goal)
    draw(grid, path, start, goal) 

    for t in range(5):  # 5번 실시간 장애물 추가 & 재탐색
        print(f"\n:시계_3시: Step {t+1}: 장애물 갱신 및 경로 재탐색")
        time.sleep(1)
        # 동적 장애물 위치 갱신 예시 (랜덤 이동)
        new_dynamic_obstacles = []
        for (x, y) in dynamic_obstacles:
            dx, dy = random.choice([(-1,0), (1,0), (0,-1), (0,1)])
            new_x, new_y = x + dx, y + dy
            if 0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]):
                new_dynamic_obstacles.append((new_x, new_y))
        dynamic_obstacles = new_dynamic_obstacles
        # 기존 경로 위에 동적 장애물이 겹치는지 확인
        if any(pos in dynamic_obstacles for pos in path):
            print(":경고: 경로에 동적 장애물 등장! 경로 재계산 중...")
            for (x, y) in dynamic_obstacles:
                grid[x][y] = 1  # 맵에 장애물로 반영
            path = find_path_with_waypoints(grid, start, waypoints, goal)
            draw(grid, path, start, goal, waypoints)

def draw(grid, path=None, start=None, goal=None, waypoints=None, dynamic_obstacles=None):
    plt.figure(figsize=(6, 6))

    # 맵 표시
    for x in range(len(grid)):
        for y in range(len(grid[0])):
            if grid[x][y] == 1:
                plt.plot(y, len(grid) - x - 1, 'ks')  # 장애물: 검정 사각형
            else:
                plt.plot(y, len(grid) - x - 1, 'ws')  # 빈칸: 흰 사각형
    # 경로
    if path:
        for (x, y) in path:
            plt.plot(y, len(grid) - x - 1, 'go')  # 경로: 초록 원
    # 시작점
    if start:
        plt.plot(start[1], len(grid) - start[0] - 1, 'bo', label='Start')  # 파란 원
    # 목적지
    if goal:
        plt.plot(goal[1], len(grid) - goal[0] - 1, 'ro', label='Goal')  # 빨간 원
    # 경유지
    if waypoints:
        for i, (x, y) in enumerate(waypoints):
            plt.plot(y, len(grid) - x - 1, 'yo')  # 노란 원
            plt.text(y + 0.2, len(grid) - x - 1 + 0.2, f'W{i+1}', fontsize=8, color='orange')
    # 동적 장애물
    if dynamic_obstacles:
        for (x, y) in dynamic_obstacles:
            plt.plot(y, len(grid) - x - 1, 'rx')  # 동적 장애물: 빨간 X
    
    plt.grid(True)
    plt.legend()
    plt.title("A* Pathfinding with Waypoints")
    plt.show()

# main 함수
if __name__ == "__main__":
    size = 10
    grid = [[0 for _ in range(size)] for _ in range(size)]
    start = (0, 0)
    waypoints = [(9, 0), (5, 1), (2, 8)]  # 경유지 리스트
    goal = (9, 9)
    dynamic_obstacles = [(3, 3), (8,2)]
    
    # 장애물 생성
    generate_random_obstacles(grid, 10)

    path = find_path_with_waypoints(grid, start, waypoints, goal)
    if path:
        draw(grid, path, start, goal, waypoints, dynamic_obstacles)
    else:
        print(":경고: 경로 없음!")