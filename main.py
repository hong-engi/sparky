from spike import MotorPair, DistanceSensor, ColorSensor
import time

# ==========================================
# 1. 상수 정의 (캘리브레이션 필요)
# ==========================================
BLOCK_DISTANCE = 25             # 1칸 직진 거리 (cm)
TURN_90_DEGREE_AMOUNT = 12.5    # 90도 회전을 위한 바퀴 이동 거리 (cm)

OBSTACLE_MIN_DIST = 5           # 장애물 감지 최소 거리 (cm)
OBSTACLE_MAX_DIST = 15          # 장애물 감지 최대 거리 (cm) (보통 1칸 앞을 볼 수 있게 설정)

# 틱 기반 감지 설정
SENSOR_TICK_COUNT = 10
SENSOR_THRESHOLD = 0.8

# 그리드 및 로봇 상태
GRID_ROWS = 4
GRID_COLS = 6
current_pos = (1, 1)  # 현재 위치 (Row, Col)
current_dir = 1       # 0:북, 1:동, 2:남, 3:서

known_obstacles = set()
pink_cells = set()

# ==========================================
# 2. 기본 제어 및 센서 함수
# ==========================================
def move_distance(motor_pair, amount, unit='cm', speed=50):
    motor_pair.move(amount, unit, steering=0, speed=speed)

def rotate_robot(motor_pair, amount, unit='cm', steering=100, speed=50):
    motor_pair.move(amount, unit, steering=steering, speed=speed)

def is_obstacle_confirmed(distance_sensor, min_cm, max_cm):
    detect_count = 0
    for _ in range(SENSOR_TICK_COUNT):
        dist = distance_sensor.get_distance_cm()
        if dist is not None and min_cm <= dist <= max_cm:
            detect_count += 1
        time.sleep(0.01)
    return (detect_count / SENSOR_TICK_COUNT) >= SENSOR_THRESHOLD

def is_color_confirmed(color_sensor, target_color):
    detect_count = 0
    for _ in range(SENSOR_TICK_COUNT):
        if color_sensor.get_color() == target_color:
            detect_count += 1
        time.sleep(0.01)
    return (detect_count / SENSOR_TICK_COUNT) >= SENSOR_THRESHOLD

# ==========================================
# 3. 네비게이션 및 알고리즘 함수
# ==========================================
def find_bfs_path(start, goal, obstacles):
    """BFS를 이용해 장애물을 피하는 최단 경로 탐색"""
    queue = [[start]]
    visited = set([start])
    
    while queue:
        path = queue.pop(0)
        node = path[-1]
        
        if node == goal:
            return path
            
        r, c = node
        neighbors = [(r-1, c), (r, c+1), (r+1, c), (r, c-1)] # 북, 동, 남, 서
        
        for nr, nc in neighbors:
            if 1 <= nr <= GRID_ROWS and 1 <= nc <= GRID_COLS:
                if (nr, nc) not in obstacles and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    queue.append(list(path) + [(nr, nc)])
    return []

def turn_towards(motor_pair, target_cell):
    global current_dir
    cr, cc = current_pos
    tr, tc = target_cell
    
    if tr < cr: target_dir = 0     # 북
    elif tc > cc: target_dir = 1   # 동
    elif tr > cr: target_dir = 2   # 남
    elif tc < cc: target_dir = 3   # 서
    else: return
    
    diff = (target_dir - current_dir) % 4
    if diff == 1:
        rotate_robot(motor_pair, TURN_90_DEGREE_AMOUNT, steering=100) # 우 90도
    elif diff == 2:
        rotate_robot(motor_pair, TURN_90_DEGREE_AMOUNT * 2, steering=100) # 180도 유턴
    elif diff == 3:
        rotate_robot(motor_pair, TURN_90_DEGREE_AMOUNT, steering=-100) # 좌 90도
        
    current_dir = target_dir

def sense_and_record_color(color_sensor):
    if is_color_confirmed(color_sensor, 'pink'):
        pink_cells.add(current_pos)
        print("분홍색 칸 발견:", current_pos)

# ==========================================
# 4. 메인 실행 함수
# ==========================================
def run_exploration(motor_pair, distance_sensor, color_sensor):
    global current_pos
    
    # 지그재그 목표 큐 생성
    target_queue = []
    for r in range(1, GRID_ROWS + 1):
        if r % 2 != 0:
            for c in range(1, GRID_COLS + 1): target_queue.append((r, c))
        else:
            for c in range(GRID_COLS, 0, -1): target_queue.append((r, c))
            
    sense_and_record_color(color_sensor)
    
    # [탐색 페이즈]
    while target_queue:
        # 조기 종료 조건
        if len(known_obstacles) >= 2 and len(pink_cells) >= 2:
            print("모든 타겟 발견! 조기 복귀 시작.")
            break
            
        target = target_queue[0]
        
        # 현재 위치가 타겟이거나 타겟이 장애물이면 큐에서 제거
        if target in known_obstacles:
            target_queue.pop(0)
            continue
            
        path = find_bfs_path(current_pos, target, known_obstacles)
        if not path:
            target_queue.pop(0)
            continue
            
        next_cell = path[1]
        turn_towards(motor_pair, next_cell)
        
        if is_obstacle_confirmed(distance_sensor, OBSTACLE_MIN_DIST, OBSTACLE_MAX_DIST):
            known_obstacles.add(next_cell)
            print("장애물 발견! 위치:", next_cell)
            continue # 경로 재탐색
            
        move_distance(motor_pair, BLOCK_DISTANCE)
        current_pos = next_cell
        
        # 우회 중 방문한 칸이 큐에 있다면 중복 방문 방지를 위해 제거
        if current_pos in target_queue:
            target_queue.remove(current_pos)
            
        sense_and_record_color(color_sensor)

    # [복귀 페이즈]
    print("복귀 시작: (1,1)로 이동")
    target_queue.clear()
    target_queue.append((1, 1))
    
    while target_queue:
        target = target_queue[0]
        path = find_bfs_path(current_pos, target, known_obstacles)
        
        if not path or current_pos == target:
            break
            
        next_cell = path[1]
        turn_towards(motor_pair, next_cell)
        
        if is_obstacle_confirmed(distance_sensor, OBSTACLE_MIN_DIST, OBSTACLE_MAX_DIST):
            known_obstacles.add(next_cell)
            continue
            
        move_distance(motor_pair, BLOCK_DISTANCE)
        current_pos = next_cell

    print("=== 미션 종료 ===")
    print("분홍색 칸:", pink_cells)
    print("장애물 칸:", known_obstacles)

# ==========================================
# 5. 하드웨어 초기화 및 실행 (주석 해제 후 사용)
# ==========================================
# hub = PrimeHub()
# motor_pair = MotorPair('B', 'A')
# dist_sensor = DistanceSensor('C')
# col_sensor = ColorSensor('D')
# run_exploration(motor_pair, dist_sensor, col_sensor)