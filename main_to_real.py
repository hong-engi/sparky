from spike import MotorPair, DistanceSensor, ColorSensor
import time
import math

# ==========================================
# 0. 로그 유틸
# ==========================================
LOG_STEP = 0

def log(msg):
    global LOG_STEP
    LOG_STEP += 1
    print("[{:03d}] {}".format(LOG_STEP, msg))

def dir_to_str(d):
    if d == 0:
        return "북"
    elif d == 1:
        return "동"
    elif d == 2:
        return "남"
    elif d == 3:
        return "서"
    return "?"

# ==========================================
# 1. 상수 정의
# ==========================================
CELL_SIZE = 25
WHEEL_DISTANCE = 13

BLOCK_DISTANCE = CELL_SIZE
TURN_90_DEGREE_AMOUNT = math.pi / 2 * (WHEEL_DISTANCE / 2)   # 약 3.93cm

OBSTACLE_MIN_DIST = 5
OBSTACLE_MAX_DIST = 15

SENSOR_TICK_COUNT = 10
SENSOR_THRESHOLD = 0.8

GRID_ROWS = 4
GRID_COLS = 6

current_pos = (1, 1)   # 현재 위치 (Row, Col)
current_dir = 1        # 0:북, 1:동, 2:남, 3:서

known_obstacles = set()
red_cells = set()

# ==========================================
# 2. 기본 제어 및 센서 함수
# ==========================================
def move_distance(motor_pair, amount, unit='cm', speed=50):
    log("직진 시작: 거리={} {}, 속도={}".format(amount, unit, speed))
    motor_pair.move(amount, unit, steering=0, speed=speed)
    log("직진 완료")

def rotate_robot(motor_pair, amount, unit='cm', steering=100, speed=50):
    direction_text = "우회전" if steering > 0 else "좌회전"
    log("회전 시작: {}, 거리={} {}, 속도={}".format(direction_text, amount, unit, speed))
    motor_pair.move(amount, unit, steering=steering, speed=speed)
    log("회전 완료: {}".format(direction_text))

def is_obstacle_confirmed(distance_sensor, min_cm, max_cm):
    detect_count = 0
    log("장애물 확인 시작: 범위={}~{}cm, 샘플 수={}".format(min_cm, max_cm, SENSOR_TICK_COUNT))

    for i in range(SENSOR_TICK_COUNT):
        dist = distance_sensor.get_distance_cm(short_range=False)
        log("  거리센서[{}] = {}".format(i + 1, dist))

        if dist is not None and min_cm <= dist <= max_cm:
            detect_count += 1

        time.sleep(0.01)

    ratio = detect_count / SENSOR_TICK_COUNT
    result = ratio >= SENSOR_THRESHOLD

    log("장애물 판정: 감지횟수={}/{}, 비율={:.2f}, 기준={:.2f}, 결과={}".format(
        detect_count, SENSOR_TICK_COUNT, ratio, SENSOR_THRESHOLD, result
    ))
    return result

def is_color_confirmed(color_sensor, target_color):
    detect_count = 0
    log("색상 확인 시작: 목표색={}, 샘플 수={}".format(target_color, SENSOR_TICK_COUNT))

    for i in range(SENSOR_TICK_COUNT):
        color = color_sensor.get_color()
        log("  컬러센서[{}] = {}".format(i + 1, color))

        if color == target_color:
            detect_count += 1

        time.sleep(0.01)

    ratio = detect_count / SENSOR_TICK_COUNT
    result = ratio >= SENSOR_THRESHOLD

    log("색상 판정: 감지횟수={}/{}, 비율={:.2f}, 기준={:.2f}, 결과={}".format(
        detect_count, SENSOR_TICK_COUNT, ratio, SENSOR_THRESHOLD, result
    ))
    return result

# ==========================================
# 3. 네비게이션 및 알고리즘 함수
# ==========================================
def find_bfs_path(start, goal, obstacles):
    log("BFS 시작: start={}, goal={}, obstacles={}".format(start, goal, sorted(list(obstacles))))
    queue = [[start]]
    visited = set([start])

    while queue:
        path = queue.pop(0)
        node = path[-1]
        log("  BFS 탐색 중: 현재 path={}".format(path))

        if node == goal:
            log("BFS 성공: 경로={}".format(path))
            return path

        r, c = node
        neighbors = [
            (r - 1, c),  # 북
            (r, c + 1),  # 동
            (r + 1, c),  # 남
            (r, c - 1)   # 서
        ]

        for nr, nc in neighbors:
            if 1 <= nr <= GRID_ROWS and 1 <= nc <= GRID_COLS:
                if (nr, nc) not in obstacles and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    queue.append(path + [(nr, nc)])
                    log("    BFS enqueue: {}".format(path + [(nr, nc)]))
                else:
                    log("    BFS skip(방문/장애물): ({}, {})".format(nr, nc))
            else:
                log("    BFS skip(맵밖): ({}, {})".format(nr, nc))

    log("BFS 실패: 갈 수 있는 경로 없음")
    return []

def turn_towards(motor_pair, target_cell):
    global current_dir, current_pos

    cr, cc = current_pos
    tr, tc = target_cell

    log("방향 전환 판단: 현재위치={}, 목표다음칸={}, 현재방향={}".format(
        current_pos, target_cell, dir_to_str(current_dir)
    ))

    if tr < cr:
        target_dir = 0   # 북
    elif tc > cc:
        target_dir = 1   # 동
    elif tr > cr:
        target_dir = 2   # 남
    elif tc < cc:
        target_dir = 3   # 서
    else:
        log("방향 전환 불필요: 같은 칸")
        return

    diff = (target_dir - current_dir) % 4

    log("목표방향={}, diff={}".format(dir_to_str(target_dir), diff))

    if diff == 1:
        log("우회전 90도 수행")
        rotate_robot(motor_pair, TURN_90_DEGREE_AMOUNT, steering=100)
    elif diff == 2:
        log("180도 회전 수행")
        rotate_robot(motor_pair, TURN_90_DEGREE_AMOUNT * 2, steering=100)
    elif diff == 3:
        log("좌회전 90도 수행")
        rotate_robot(motor_pair, TURN_90_DEGREE_AMOUNT, steering=-100)
    else:
        log("회전 없음: 이미 목표 방향")

    current_dir = target_dir
    log("방향 갱신 완료: 현재방향={}".format(dir_to_str(current_dir)))

def sense_and_record_color(color_sensor):
    log("현재 칸 색상 검사: 위치={}".format(current_pos))
    if is_color_confirmed(color_sensor, 'red'):
        if current_pos not in red_cells:
            red_cells.add(current_pos)
            log("분홍색 칸 새로 기록: {}".format(current_pos))
        else:
            log("분홍색 칸 재확인: {}".format(current_pos))
    else:
        log("분홍색 칸 아님: {}".format(current_pos))

# ==========================================
# 4. 메인 실행 함수
# ==========================================
def run_exploration(motor_pair, distance_sensor, color_sensor):
    global current_pos

    log("탐색 시작")
    log("초기 상태: current_pos={}, current_dir={}".format(current_pos, dir_to_str(current_dir)))
    log("격자 크기: {}행 x {}열".format(GRID_ROWS, GRID_COLS))

    # 지그재그 목표 큐 생성
    target_queue = []
    for r in range(1, GRID_ROWS + 1):
        if r % 2 != 0:
            for c in range(1, GRID_COLS + 1):
                target_queue.append((r, c))
        else:
            for c in range(GRID_COLS, 0, -1):
                target_queue.append((r, c))

    log("초기 목표 큐 생성 완료: {}".format(target_queue))

    sense_and_record_color(color_sensor)

    # [탐색 페이즈]
    while target_queue:
        log("----- 탐색 루프 시작 -----")
        log("현재 위치={}, 방향={}".format(current_pos, dir_to_str(current_dir)))
        log("남은 목표 큐={}".format(target_queue))
        log("현재 장애물={}, 현재 분홍칸={}".format(sorted(list(known_obstacles)), sorted(list(red_cells))))

        if len(known_obstacles) >= 2 and len(red_cells) >= 2:
            log("모든 타겟 발견 조건 충족 -> 조기 복귀")
            break

        target = target_queue[0]
        log("현재 목표 칸={}".format(target))

        if target in known_obstacles:
            log("목표 칸이 장애물로 판명되어 큐에서 제거: {}".format(target))
            target_queue.pop(0)
            continue

        path = find_bfs_path(current_pos, target, known_obstacles)

        if not path:
            log("경로 없음 -> 목표 제거: {}".format(target))
            target_queue.pop(0)
            continue

        if len(path) < 2:
            log("이미 목표 칸이거나 이동 필요 없음 -> 목표 제거: {}".format(target))
            target_queue.pop(0)
            continue

        next_cell = path[1]
        log("다음 이동 칸={}".format(next_cell))

        turn_towards(motor_pair, next_cell)

        if is_obstacle_confirmed(distance_sensor, OBSTACLE_MIN_DIST, OBSTACLE_MAX_DIST):
            known_obstacles.add(next_cell)
            log("장애물 발견 -> 기록 후 재탐색: {}".format(next_cell))
            continue

        log("전방 장애물 없음 -> 이동 수행")
        move_distance(motor_pair, BLOCK_DISTANCE)
        current_pos = next_cell
        log("이동 후 현재 위치 갱신: {}".format(current_pos))

        if current_pos in target_queue:
            target_queue.remove(current_pos)
            log("도착한 칸을 목표 큐에서 제거: {}".format(current_pos))

        sense_and_record_color(color_sensor)

    # [복귀 페이즈]
    log("복귀 시작: 목표=(1,1)")
    target_queue = [(1, 1)]

    while target_queue:
        log("----- 복귀 루프 시작 -----")
        log("현재 위치={}, 방향={}".format(current_pos, dir_to_str(current_dir)))
        log("현재 장애물={}".format(sorted(list(known_obstacles))))

        target = target_queue[0]
        path = find_bfs_path(current_pos, target, known_obstacles)

        if not path or current_pos == target:
            log("복귀 종료 조건 충족: path={}, current_pos={}".format(path, current_pos))
            break

        if len(path) < 2:
            log("복귀 경로 길이 부족 -> 종료")
            break

        next_cell = path[1]
        log("복귀 다음 칸={}".format(next_cell))

        turn_towards(motor_pair, next_cell)

        if is_obstacle_confirmed(distance_sensor, OBSTACLE_MIN_DIST, OBSTACLE_MAX_DIST):
            known_obstacles.add(next_cell)
            log("복귀 중 새 장애물 발견 -> 기록: {}".format(next_cell))
            continue

        log("복귀 이동 수행")
        move_distance(motor_pair, BLOCK_DISTANCE)
        current_pos = next_cell
        log("복귀 후 현재 위치 갱신: {}".format(current_pos))

    log("=== 미션 종료 ===")
    log("최종 분홍색 칸={}".format(sorted(list(red_cells))))
    log("최종 장애물 칸={}".format(sorted(list(known_obstacles))))
    log("최종 위치={}, 최종 방향={}".format(current_pos, dir_to_str(current_dir)))

# ==========================================
# 5. 하드웨어 초기화 및 실행
# ==========================================
log("하드웨어 초기화 시작")
motor_pair = MotorPair('B', 'A')
dist_sensor = DistanceSensor('C')
col_sensor = ColorSensor('D')
log("하드웨어 초기화 완료")

run_exploration(motor_pair, dist_sensor, col_sensor)