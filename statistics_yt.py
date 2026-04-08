import math
import random
import time

# ==========================================
# 환경 설정
# ==========================================
CELL_SIZE_CM = 23.0
GRID_SIZE_X = 6
GRID_SIZE_Y = 4
BOX_SIZE_CM = 15.0
LIMIT_DISTANCE = CELL_SIZE_CM * 6

OFFSET_X = 0.0
OFFSET_Y = 0.0

ROTATE_STEP_DEG = 0.5
WHEEL_DISTANCE = 5.0

# 시간 파라미터(예시값)
TIME_PER_MOVE = 1.2
TIME_PER_90_TURN = 0.8
TIME_PER_DISTANCE_READ = 0.02
TIME_PER_SCAN_CALL_OVERHEAD = 0.0

MIN_BOXES = 1
MAX_BOXES = 2

# ==========================================
# 유틸
# ==========================================
def normalize_angle(rad):
    while rad > math.pi:
        rad -= 2 * math.pi
    while rad <= -math.pi:
        rad += 2 * math.pi
    return rad

def cell_center_cm(cx, cy):
    return ((cx + 0.5) * CELL_SIZE_CM, (cy + 0.5) * CELL_SIZE_CM)

def ray_aabb_intersection(ox, oy, dx, dy, min_x, max_x, min_y, max_y):
    eps = 1e-9

    if abs(dx) < eps:
        if ox < min_x or ox > max_x:
            return None
        tx_min = float("-inf")
        tx_max = float("inf")
    else:
        tx1 = (min_x - ox) / dx
        tx2 = (max_x - ox) / dx
        tx_min = min(tx1, tx2)
        tx_max = max(tx1, tx2)

    if abs(dy) < eps:
        if oy < min_y or oy > max_y:
            return None
        ty_min = float("-inf")
        ty_max = float("inf")
    else:
        ty1 = (min_y - oy) / dy
        ty2 = (max_y - oy) / dy
        ty_min = min(ty1, ty2)
        ty_max = max(ty1, ty2)

    t_enter = max(tx_min, ty_min)
    t_exit = min(tx_max, ty_max)

    if t_exit < 0 or t_enter > t_exit:
        return None

    return t_enter if t_enter >= 0 else t_exit

# ==========================================
# 환경
# ==========================================
class SimEnvironment:
    def __init__(self, box_cells):
        self.box_cells = list(box_cells)

    def raycast_distance(self, robot_x, robot_y, heading_rad):
        closest = None
        dx = math.sin(heading_rad)
        dy = math.cos(heading_rad)

        for cx, cy in self.box_cells:
            center_x, center_y = cell_center_cm(cx, cy)
            half = BOX_SIZE_CM / 2.0

            min_x = center_x - half
            max_x = center_x + half
            min_y = center_y - half
            max_y = center_y + half

            dist = ray_aabb_intersection(robot_x, robot_y, dx, dy, min_x, max_x, min_y, max_y)
            if dist is None or dist < 0 or dist > LIMIT_DISTANCE:
                continue

            if closest is None or dist < closest:
                closest = dist

        return closest

# ==========================================
# 로봇
# ==========================================
class SimRobot:
    def __init__(self, env):
        self.env = env

        self.robot_x = CELL_SIZE_CM / 2.0
        self.robot_y = CELL_SIZE_CM / 2.0
        self.current_rad = 0.0  # +y 방향

        self.box_coordinates = []
        self.box_count = 0

        self.move_count = 0
        self.turn_count_90 = 0.0
        self.scan_count = 0
        self.distance_read_count = 0

    def move_one_cell(self, direction=1):
        self.robot_x += CELL_SIZE_CM * math.sin(self.current_rad) * direction
        self.robot_y += CELL_SIZE_CM * math.cos(self.current_rad) * direction
        self.move_count += 1

    def turn(self, angle_rad):
        self.current_rad = normalize_angle(self.current_rad + angle_rad)
        self.turn_count_90 += abs(angle_rad) / (math.pi / 2.0)

    def turn_right(self):
        self.turn(math.pi / 2.0)

    def scan(self, end_deg=90):
        self.scan_count += 1
        direction = 1 if end_deg > 0 else -1
        step_count = int(abs(end_deg) / ROTATE_STEP_DEG)

        for _ in range(step_count):
            self.turn(math.radians(direction * ROTATE_STEP_DEG))

            dist_cm = self.env.raycast_distance(self.robot_x, self.robot_y, self.current_rad)
            self.distance_read_count += 1

            if dist_cm is None or dist_cm > LIMIT_DISTANCE:
                continue

            x = (dist_cm + OFFSET_Y) * math.sin(self.current_rad) + OFFSET_X * math.cos(self.current_rad) + self.robot_x
            y = (dist_cm + OFFSET_Y) * math.cos(self.current_rad) - OFFSET_X * math.sin(self.current_rad) + self.robot_y

            cx = int(x // CELL_SIZE_CM)
            cy = int(y // CELL_SIZE_CM)

            if 0 <= cx < GRID_SIZE_X and 0 <= cy < GRID_SIZE_Y:
                if (cx, cy) not in self.box_coordinates:
                    self.box_coordinates.append((cx, cy))
                    self.box_count += 1

    def run_friend_algorithm(self):
        self.scan(90)

        if self.box_count == 1:
            if self.box_coordinates[0][0] == 0:
                self.move_one_cell()
                self.scan(-180)
            else:
                self.turn_right()
                self.move_one_cell(direction=-1)
                self.scan(-90)
                if self.box_count == 1:
                    self.scan(-90)

        return sorted(self.box_coordinates)

# ==========================================
# 평가
# ==========================================
def generate_random_boxes():
    candidates = [(x, y) for x in range(GRID_SIZE_X) for y in range(GRID_SIZE_Y)]
    if (0, 0) in candidates:
        candidates.remove((0, 0))
    random.shuffle(candidates)
    n_boxes = random.randint(MIN_BOXES, MAX_BOXES)
    return candidates[:n_boxes]

def evaluate_once():
    true_boxes = sorted(generate_random_boxes())
    env = SimEnvironment(true_boxes)
    robot = SimRobot(env)
    estimated_boxes = robot.run_friend_algorithm()

    true_set = set(true_boxes)
    est_set = set(estimated_boxes)

    exact_match = (true_set == est_set)
    precision = len(true_set & est_set) / len(est_set) if len(est_set) > 0 else 0.0
    recall = len(true_set & est_set) / len(true_set) if len(true_set) > 0 else 0.0

    est_time = (
        robot.move_count * TIME_PER_MOVE
        + robot.turn_count_90 * TIME_PER_90_TURN
        + robot.distance_read_count * TIME_PER_DISTANCE_READ
        + robot.scan_count * TIME_PER_SCAN_CALL_OVERHEAD
    )

    return {
        "true_boxes": true_boxes,
        "estimated_boxes": estimated_boxes,
        "exact_match": exact_match,
        "precision": precision,
        "recall": recall,
        "move_count": robot.move_count,
        "turn_count_90": robot.turn_count_90,
        "scan_count": robot.scan_count,
        "distance_read_count": robot.distance_read_count,
        "est_time": est_time,
    }

def evaluate_many(n_runs=10000):
    print(f"[{n_runs}회 시뮬레이션 시작]")
    start = time.time()

    total_exact = 0
    total_precision = 0.0
    total_recall = 0.0

    total_moves = 0
    total_turn90 = 0.0
    total_scans = 0
    total_reads = 0
    total_time = 0.0

    failures = []

    progress_step = max(1, n_runs // 10)

    for i in range(n_runs):
        result = evaluate_once()

        total_exact += int(result["exact_match"])
        total_precision += result["precision"]
        total_recall += result["recall"]

        total_moves += result["move_count"]
        total_turn90 += result["turn_count_90"]
        total_scans += result["scan_count"]
        total_reads += result["distance_read_count"]
        total_time += result["est_time"]

        if (not result["exact_match"]) and len(failures) < 5:
            failures.append((result["true_boxes"], result["estimated_boxes"]))

        if (i + 1) % progress_step == 0:
            print(f"진행률: {(i + 1) / n_runs * 100:.0f}%")

    end = time.time()

    avg_exact = total_exact / n_runs
    avg_precision = total_precision / n_runs
    avg_recall = total_recall / n_runs

    avg_moves = total_moves / n_runs
    avg_turn90 = total_turn90 / n_runs
    avg_scans = total_scans / n_runs
    avg_reads = total_reads / n_runs
    avg_time = total_time / n_runs

    print("\n" + "=" * 60)
    print("새 알고리즘 Statistics 결과")
    print("=" * 60)
    print(f"실행 횟수: {n_runs}")
    print(f"Exact Match: {avg_exact * 100:.2f}%")
    print(f"평균 Precision: {avg_precision * 100:.2f}%")
    print(f"평균 Recall: {avg_recall * 100:.2f}%")
    print("-" * 60)
    print(f"평균 이동 횟수: {avg_moves:.2f}")
    print(f"평균 90도 회전 환산 횟수: {avg_turn90:.2f}")
    print(f"평균 스캔 호출 횟수: {avg_scans:.2f}")
    print(f"평균 거리센서 측정 횟수: {avg_reads:.2f}")
    print("-" * 60)
    print(f"예상 평균 소요 시간: {avg_time:.2f}초")
    print(f"시뮬레이션 계산 시간: {end - start:.3f}초")
    print("=" * 60)

    if failures:
        print("\n[실패 예시 최대 5개]")
        for idx, (true_boxes, estimated_boxes) in enumerate(failures, 1):
            print(f"{idx}. 실제: {true_boxes}, 추정: {estimated_boxes}")

# ==========================================
# 실행부
# ==========================================
if __name__ == "__main__":
    N_RUNS = 10000
    evaluate_many(N_RUNS)