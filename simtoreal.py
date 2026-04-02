import tkinter as tk
import time
import random

# ==========================================
# 시뮬레이터 설정
# ==========================================
ROWS, COLS = 4, 6
CELL_SIZE = 80  # 픽셀 단위 칸 크기
OFFSET = 20     # 여백

class RobotSimulator:
    def __init__(self, root):
        self.root = root
        self.root.title("로봇 자율주행 샌드박스 시뮬레이터")
        self.canvas = tk.Canvas(root, width=COLS*CELL_SIZE + OFFSET*2, height=ROWS*CELL_SIZE + OFFSET*2, bg='white')
        self.canvas.pack()

        # 1. 샌드박스 실제 환경 생성 (로봇은 아직 모르는 정보)
        all_cells = [(r, c) for r in range(1, ROWS+1) for c in range(1, COLS+1) if (r, c) != (1, 1)]
        random.shuffle(all_cells)
        self.real_obstacles = set(all_cells[:2])     # 실제 장애물 2개
        self.real_pink_cells = set(all_cells[2:4])   # 실제 분홍색 칸 2개

        # 2. 로봇의 두뇌 상태 (로봇이 탐색하며 알아가는 정보)
        self.known_obstacles = set()
        self.known_pink_cells = set()
        self.current_pos = (1, 1)
        self.current_dir = 1 # 0:북, 1:동, 2:남, 3:서
        self.trace = [(1, 1)] # 이동 궤적 (Trace) 저장

        # 3. 지그재그 목표 큐 생성
        self.target_queue = []
        for r in range(1, ROWS + 1):
            if r % 2 != 0:
                for c in range(1, COLS + 1): self.target_queue.append((r, c))
            else:
                for c in range(COLS, 0, -1): self.target_queue.append((r, c))

        self.draw_grid()
        
        # UI가 뜨고 1초 뒤에 자동으로 시뮬레이션 시작
        self.root.after(1000, self.run_simulation)

    def get_coords(self, r, c):
        """그리드 좌표(Row, Col)를 픽셀 좌표(X, Y)로 변환"""
        x = OFFSET + (c - 1) * CELL_SIZE
        y = OFFSET + (r - 1) * CELL_SIZE
        return x, y

    def draw_grid(self):
        """현재 상태를 캔버스에 그리는 함수"""
        self.canvas.delete("all")
        
        # 1. 그리드 및 타겟 그리기
        for r in range(1, ROWS + 1):
            for c in range(1, COLS + 1):
                x, y = self.get_coords(r, c)
                color = "white"

                # 시뮬레이터 상에 숨겨진 실제 타겟 표시 (연한 색)
                if (r, c) in self.real_obstacles: color = "#e0e0e0" # 연한 회색
                elif (r, c) in self.real_pink_cells: color = "#ffe4e1" # 연한 분홍

                # 로봇이 센서로 발견한 타겟 표시 (진한 색으로 덮어쓰기)
                if (r, c) in self.known_obstacles: color = "#333333" # 진한 회색/검정
                elif (r, c) in self.known_pink_cells: color = "#ff1493" # 진한 분홍

                self.canvas.create_rectangle(x, y, x+CELL_SIZE, y+CELL_SIZE, fill=color, outline="gray")

        # 2. 이동 궤적(Trace) 그리기
        if len(self.trace) > 1:
            trace_coords = []
            for tr, tc in self.trace:
                tx, ty = self.get_coords(tr, tc)
                trace_coords.extend([tx + CELL_SIZE/2, ty + CELL_SIZE/2])
            # 빨간색 점선으로 궤적 표시
            self.canvas.create_line(trace_coords, fill="red", width=3, dash=(5, 2))

        # 3. 로봇 본체 및 방향 그리기
        rx, ry = self.get_coords(self.current_pos[0], self.current_pos[1])
        cx, cy = rx + CELL_SIZE/2, ry + CELL_SIZE/2
        self.canvas.create_oval(cx - 20, cy - 20, cx + 20, cy + 20, fill="#4285F4") # 파란색 원

        # 4. 방향 지시선 (노란색 선으로 바라보는 방향 표시)
        dir_dx = [0, 20, 0, -20]
        dir_dy = [-20, 0, 20, 0]
        self.canvas.create_line(cx, cy, cx + dir_dx[self.current_dir], cy + dir_dy[self.current_dir], fill="yellow", width=3)

        self.root.update()

    def find_bfs_path(self, start, goal):
        """BFS 최단 경로 탐색"""
        queue = [[start]]
        visited = set([start])
        while queue:
            path = queue.pop(0)
            node = path[-1]
            if node == goal: return path
            r, c = node
            for nr, nc in [(r-1, c), (r, c+1), (r+1, c), (r, c-1)]:
                if 1 <= nr <= ROWS and 1 <= nc <= COLS:
                    # '로봇이 알고 있는 장애물'만 피해서 탐색 (실제 장애물은 부딪혀봐야 앎)
                    if (nr, nc) not in self.known_obstacles and (nr, nc) not in visited:
                        visited.add((nr, nc))
                        queue.append(list(path) + [(nr, nc)])
        return []

    def turn_towards(self, target_cell):
        """목표 칸을 바라보도록 회전"""
        cr, cc = self.current_pos
        tr, tc = target_cell
        if tr < cr: self.current_dir = 0     # 북
        elif tc > cc: self.current_dir = 1   # 동
        elif tr > cr: self.current_dir = 2   # 남
        elif tc < cc: self.current_dir = 3   # 서
        self.draw_grid()
        time.sleep(0.2) # 회전하는 시간 묘사

    def sense_color(self):
        """현재 칸의 색상 감지 시뮬레이션"""
        if self.current_pos in self.real_pink_cells:
            self.known_pink_cells.add(self.current_pos)
            self.draw_grid()

    def run_simulation(self):
        """메인 네비게이션 로직 (실제 스파이크 코드와 동일한 구조)"""
        self.sense_color()

        while self.target_queue:
            # 조기 종료 조건 검사
            if len(self.known_obstacles) >= 2 and len(self.known_pink_cells) >= 2:
                print("모든 타겟을 발견했습니다! 복귀를 시작합니다.")
                break

            target = self.target_queue[0]

            if target in self.known_obstacles:
                self.target_queue.pop(0)
                continue

            path = self.find_bfs_path(self.current_pos, target)
            if not path:
                self.target_queue.pop(0)
                continue

            if self.current_pos == target:
                self.target_queue.pop(0)
                continue

            # 다음 1칸
            next_cell = path[1]
            self.turn_towards(next_cell)

            # 이동 전 거리 센서 시뮬레이션 (앞 칸이 실제 장애물인지 확인)
            if next_cell in self.real_obstacles:
                print(f"장애물 감지! 위치: {next_cell}")
                self.known_obstacles.add(next_cell)
                self.draw_grid()
                time.sleep(0.3)
                continue # 우회로 재탐색

            # 장애물이 없으면 이동
            self.current_pos = next_cell
            self.trace.append(self.current_pos) # 궤적 추가
            
            # 우회 중 타겟 큐에 있는 칸을 밟았다면 중복 방문 방지
            if self.current_pos in self.target_queue:
                self.target_queue.remove(self.current_pos)

            self.draw_grid()
            self.sense_color()
            time.sleep(0.3) # 이동 속도 조절

        # 복귀 페이즈
        print("복귀 페이즈 시작: (1,1)로 이동합니다.")
        self.target_queue.clear()
        self.target_queue.append((1, 1))

        while self.target_queue:
            target = self.target_queue[0]
            path = self.find_bfs_path(self.current_pos, target)
            if not path or self.current_pos == target: break

            next_cell = path[1]
            self.turn_towards(next_cell)

            if next_cell in self.real_obstacles:
                self.known_obstacles.add(next_cell)
                self.draw_grid()
                time.sleep(0.3)
                continue

            self.current_pos = next_cell
            self.trace.append(self.current_pos)
            self.draw_grid()
            time.sleep(0.3)

        print("=== 시뮬레이션 완료 ===")
        print(f"최종 발견된 분홍색 칸: {self.known_pink_cells}")
        print(f"최종 발견된 장애물: {self.known_obstacles}")

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotSimulator(root)
    root.mainloop()