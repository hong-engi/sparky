import random
import time

# ==========================================
# 1. 환경 설정
# ==========================================
GRID_ROWS = 4
GRID_COLS = 6

def find_bfs_path(start, goal, obstacles):
    """BFS 최단 경로 탐색"""
    queue = [[start]]
    visited = {start}
    
    while queue:
        path = queue.pop(0)
        node = path[-1]
        if node == goal: return path
        r, c = node
        neighbors = [(r-1, c), (r, c+1), (r+1, c), (r, c-1)]
        for nr, nc in neighbors:
            if 1 <= nr <= GRID_ROWS and 1 <= nc <= GRID_COLS:
                if (nr, nc) not in obstacles and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    queue.append(list(path) + [(nr, nc)])
    return []

# ==========================================
# 2. 단일 시뮬레이션 실행 함수
# ==========================================
def simulate_one_run():
    all_cells = [(r, c) for r in range(1, GRID_ROWS + 1) for c in range(1, GRID_COLS + 1) if (r, c) != (1, 1)]
    random.shuffle(all_cells)
    real_obstacles = set(all_cells[:2])
    real_pink_cells = set(all_cells[2:4])

    current_pos = (1, 1)
    current_dir = 1  
    
    known_obstacles = set()
    pink_cells = set()
    
    # 성과 측정용 변수 (액션별 카운트)
    turns = 0
    moves = 0
    sense_obs_count = 0
    sense_color_count = 0

    target_queue = []
    for r in range(1, GRID_ROWS + 1):
        if r % 2 != 0:
            for c in range(1, GRID_COLS + 1): target_queue.append((r, c))
        else:
            for c in range(GRID_COLS, 0, -1): target_queue.append((r, c))

    def turn_towards(cr, cc, tr, tc):
        nonlocal current_dir, turns
        if tr < cr: target_dir = 0
        elif tc > cc: target_dir = 1
        elif tr > cr: target_dir = 2
        elif tc < cc: target_dir = 3
        else: return
        
        diff = (target_dir - current_dir) % 4
        if diff == 1 or diff == 3: turns += 1  
        elif diff == 2: turns += 2  
            
        current_dir = target_dir

    def sense_color():
        nonlocal sense_color_count
        sense_color_count += 1
        if current_pos in real_pink_cells:
            pink_cells.add(current_pos)

    # 시작점 색상 확인
    sense_color()

    # ── 탐색 루프 ──
    while target_queue:
        if len(known_obstacles) >= 2 and len(pink_cells) >= 2: break
        target = target_queue[0]
        
        if target in known_obstacles:
            target_queue.pop(0)
            continue
            
        path = find_bfs_path(current_pos, target, known_obstacles)
        if not path:
            target_queue.pop(0)
            continue
            
        if current_pos == target:
            target_queue.pop(0)
            continue
            
        next_cell = path[1]
        turn_towards(current_pos[0], current_pos[1], next_cell[0], next_cell[1])
        
        # 앞 칸 장애물 확인
        sense_obs_count += 1
        if next_cell in real_obstacles:
            known_obstacles.add(next_cell)
            continue 
            
        # 전진
        current_pos = next_cell
        moves += 1
        
        if current_pos in target_queue:
            target_queue.remove(current_pos)
            
        # 색상 감지
        sense_color()

    # ── 복귀 루프 ──
    target_queue = [(1, 1)]
    while target_queue:
        target = target_queue[0]
        path = find_bfs_path(current_pos, target, known_obstacles)
        
        if not path or current_pos == target: break
            
        next_cell = path[1]
        turn_towards(current_pos[0], current_pos[1], next_cell[0], next_cell[1])
        
        sense_obs_count += 1
        if next_cell in real_obstacles:
            known_obstacles.add(next_cell)
            continue
            
        current_pos = next_cell
        moves += 1

    return turns, moves, sense_obs_count, sense_color_count

# ==========================================
# 3. N회 반복 시뮬레이션 및 시간 계산
# ==========================================
def evaluate_algorithm(runs, t_move, t_turn, t_obs, t_color):
    print(f"[{runs}회 시뮬레이션 시작]")
    start_time = time.time()
    
    tot_turns, tot_moves, tot_obs, tot_color = 0, 0, 0, 0
    
    for i in range(runs):
        turns, moves, obs, color = simulate_one_run()
        tot_turns += turns
        tot_moves += moves
        tot_obs += obs
        tot_color += color
        
        if (i + 1) % (runs // 10) == 0:
            print(f"진행률: {(i + 1) / runs * 100:.0f}%")

    end_time = time.time()
    
    # 평균 횟수 계산
    avg_turns = tot_turns / runs
    avg_moves = tot_moves / runs
    avg_obs = tot_obs / runs
    avg_color = tot_color / runs
    
    # 예상 소요 시간 계산
    est_time = (avg_moves * t_move) + (avg_turns * t_turn) + (avg_obs * t_obs) + (avg_color * t_color)
    
    print("\n" + "="*50)
    print("🎯 미션 성과 및 예상 소요 시간 분석")
    print("="*50)
    print(f"[액션별 평균 발생 횟수]")
    print(f" - 직진 이동: {avg_moves:.1f}번")
    print(f" - 90도 회전: {avg_turns:.1f}번")
    print(f" - 장애물 스캔: {avg_obs:.1f}번")
    print(f" - 색상 스캔: {avg_color:.1f}번")
    print("-" * 50)
    print(f"⏱️ 예상 총 소요 시간: {est_time:.2f} 초")
    print(f"   (시뮬레이션 연산 소요 시간: {end_time - start_time:.3f}초)")
    print("="*50)

# ==========================================
# 4. 실행부 (로봇 스펙에 맞게 시간 입력)
# ==========================================
if __name__ == "__main__":
    # 각 동작에 걸리는 실제 시간(초)을 여기에 입력하세요.
    # (예시 값이며, 실제 로봇으로 측정한 뒤 수정하세요)
    TIME_PER_MOVE = 1.2        # 1칸 직진하는 데 걸리는 시간
    TIME_PER_90_TURN = 0.8     # 90도 회전하는 데 걸리는 시간
    TIME_PER_OBS_SENSE = 0.15  # 장애물 센서(10틱) 스캔에 걸리는 시간
    TIME_PER_COLOR_SENSE = 0.15 # 컬러 센서(10틱) 스캔에 걸리는 시간
    
    N_RUNS = 10000 
    
    evaluate_algorithm(
        runs=N_RUNS, 
        t_move=TIME_PER_MOVE, 
        t_turn=TIME_PER_90_TURN, 
        t_obs=TIME_PER_OBS_SENSE, 
        t_color=TIME_PER_COLOR_SENSE
    )