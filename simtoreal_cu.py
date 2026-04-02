import tkinter as tk
import time
import numpy as np
from collections import deque

# ── 셀 상태 상수 ──────────────────────────────────────────────────────
UNKNOWN = 0
EMPTY   = 1
BOX     = 2
GOAL    = 3
HOME    = 4

# ── BFS 경로 탐색 ─────────────────────────────────────────────────────
def bfs_path(start: tuple[int, int], goal: tuple[int, int], grid_map: np.ndarray, m: int, n: int) -> list[tuple[int, int]] | None:
    if start == goal: return [start]
    queue = deque([(start, [start])])
    visited = {start}
    while queue:
        pos, path = queue.popleft()
        x, y = pos
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = x + dx, y + dy
            npos = (nx, ny)
            if (0 <= nx < m) and (0 <= ny < n) and (npos not in visited):
                if grid_map[nx, ny] != BOX:
                    new_path = path + [npos]
                    if npos == goal: return new_path
                    visited.add(npos)
                    queue.append((npos, new_path))
    return None

# ── IR 스캔 → box map 업데이트 ────────────────────────────────────────
def scan_and_update(pos: tuple[int, int], sense_fn, grid_map: np.ndarray, boxes_found: set, m: int, n: int, d: float) -> None:
    if len(boxes_found) == 2: return
    max_range = d * np.sqrt(m**2 + n**2)
    cx, cy = (pos[0] + 0.5) * d, (pos[1] + 0.5) * d

    for theta, r in sense_fn(pos):
        if r > max_range: continue 
        bx = cx + r * np.cos(theta)
        by = cy + r * np.sin(theta)
        gi, gj = int(np.floor(bx / d)), int(np.floor(by / d))
        
        if (0 <= gi < m) and (0 <= gj < n):
            if grid_map[gi, gj] not in (GOAL, HOME):
                grid_map[gi, gj] = BOX
                boxes_found.add((gi, gj))
        if len(boxes_found) == 2: return

# ── Snake 방문 순서 생성 ──────────────────────────────────────────────
def snake_order(m: int, n: int) -> list[tuple[int, int]]:
    order = []
    for j in range(n):
        i_range = range(m) if j % 2 == 0 else range(m - 1, -1, -1)
        for i in i_range:
            if (i, j) != (0, 0): order.append((i, j))
    return order

# ── 메인 순회 알고리즘 (UI 훅 추가) ───────────────────────────────────
def traverse(m: int, n: int, d: float, sense_fn, step_fn, is_goal_fn, update_ui_fn=None):
    grid_map = np.full((m, n), UNKNOWN, dtype=int)
    grid_map[0, 0] = HOME
    pos = (0, 0)
    goals_found = []
    boxes_found = set()
    visited = {pos}
    targets = snake_order(m, n)

    if update_ui_fn: update_ui_fn(grid_map, pos, list(visited))

    for target in targets:
        if target in visited or grid_map[target[0], target[1]] == BOX: continue
        path = bfs_path(pos, target, grid_map, m, n)
        if path is None: continue

        path_idx = 1
        while path_idx < len(path):
            next_pos = path[path_idx]
            if len(boxes_found) < 2:
                scan_and_update(pos, sense_fn, grid_map, boxes_found, m, n, d)
                if grid_map[next_pos[0], next_pos[1]] == BOX:
                    path = bfs_path(pos, target, grid_map, m, n)
                    if path is None: break 
                    path_idx = 1
                    continue

            step_fn(pos, next_pos)
            pos = next_pos
            visited.add(pos)

            if grid_map[pos[0], pos[1]] == UNKNOWN: grid_map[pos[0], pos[1]] = EMPTY
            if is_goal_fn(pos):
                goals_found.append(pos)
                grid_map[pos[0], pos[1]] = GOAL
                print(f"  [Goal {len(goals_found)}] 발견: {pos}")
                if len(goals_found) == 2: break
            
            if update_ui_fn: update_ui_fn(grid_map, pos, list(visited))
            path_idx += 1
        if len(goals_found) == 2: break

    print(f"\n귀환 시작: 현재 위치 {pos}")
    path_home = bfs_path(pos, (0, 0), grid_map, m, n)
    if path_home:
        for next_pos in path_home[1:]:
            step_fn(pos, next_pos)
            pos = next_pos
            if update_ui_fn: update_ui_fn(grid_map, pos, list(visited))
    else: print("[경고] 귀환 경로 없음!")
    
    return goals_found, boxes_found, grid_map

# ── 시뮬레이터 UI 클래스 ──────────────────────────────────────────────
class RobotSimulator:
    def __init__(self, root):
        self.root = root
        self.root.title("Numpy 기반 자율주행 샌드박스")
        
        # 환경 설정
        self.M, self.N, self.D = 6, 4, 1.0
        self.TRUE_BOXES = {(2, 1), (4, 3)}
        self.TRUE_GOALS = {(1, 3), (3, 2)}
        
        # UI 설정
        self.cell_size = 80
        self.offset = 20
        self.canvas = tk.Canvas(root, width=self.M*self.cell_size + self.offset*2, 
                                height=self.N*self.cell_size + self.offset*2, bg='white')
        self.canvas.pack()

        # 상태 변수
        self.current_pos = (0, 0)
        self.grid_map = np.full((self.M, self.N), UNKNOWN, dtype=int)
        self.trace = []
        
        self.draw_grid()
        self.root.after(1000, self.run_algorithm)

    def get_coords(self, i, j):
        """좌표 변환: (0,0)을 캔버스 좌측 하단으로 매핑"""
        x = self.offset + i * self.cell_size
        y = self.offset + (self.N - 1 - j) * self.cell_size
        return x, y

    def update_ui(self, grid_map, pos, visited):
        """알고리즘 동작 중 실시간으로 상태를 동기화하고 화면을 다시 그립니다."""
        self.grid_map = grid_map
        self.current_pos = pos
        if pos not in self.trace:
            self.trace.append(pos)
        self.draw_grid()
        time.sleep(0.3) # 로봇 이동 속도 묘사

    def draw_grid(self):
        self.canvas.delete("all")
        
        # 1. 그리드 셀 그리기
        for i in range(self.M):
            for j in range(self.N):
                x, y = self.get_coords(i, j)
                color = "white"

                # 실제 숨겨진 타겟 표시 (연한 색상)
                if (i, j) in self.TRUE_BOXES: color = "#e0e0e0"
                elif (i, j) in self.TRUE_GOALS: color = "#ffe4e1"

                # 알고리즘이 발견한 맵 오버레이 (진한 색상)
                state = self.grid_map[i, j]
                if state == EMPTY: color = "#f9f9f9"
                elif state == BOX: color = "#333333"
                elif state == GOAL: color = "#ff1493"
                elif state == HOME: color = "#a8e6cf"

                self.canvas.create_rectangle(x, y, x+self.cell_size, y+self.cell_size, fill=color, outline="gray")

        # 2. 이동 궤적(Trace) 그리기
        if len(self.trace) > 1:
            trace_coords = []
            for ti, tj in self.trace:
                tx, ty = self.get_coords(ti, tj)
                trace_coords.extend([tx + self.cell_size/2, ty + self.cell_size/2])
            self.canvas.create_line(trace_coords, fill="red", width=3, dash=(5, 2))

        # 3. 로봇 본체 그리기
        rx, ry = self.get_coords(self.current_pos[0], self.current_pos[1])
        cx, cy = rx + self.cell_size/2, ry + self.cell_size/2
        self.canvas.create_oval(cx - 20, cy - 20, cx + 20, cy + 20, fill="#4285F4")
        self.root.update()

    # --- 센서 및 이동 콜백 함수들 ---
    def sense_fn(self, pos):
        cx, cy = (pos[0] + 0.5) * self.D, (pos[1] + 0.5) * self.D
        results = []
        for bi, bj in self.TRUE_BOXES:
            bx, by = (bi + 0.5) * self.D, (bj + 0.5) * self.D
            r = np.hypot(bx - cx, by - cy)
            theta = np.arctan2(by - cy, bx - cx)
            results.append((theta, r))
        return results

    def step_fn(self, pos, next_pos):
        print(f"  이동: {pos} → {next_pos}")

    def is_goal_fn(self, pos):
        return pos in self.TRUE_GOALS

    def run_algorithm(self):
        print("시뮬레이션 시작!")
        traverse(self.M, self.N, self.D, self.sense_fn, self.step_fn, self.is_goal_fn, update_ui_fn=self.update_ui)
        print("=== 시뮬레이션 완료 ===")

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotSimulator(root)
    root.mainloop()