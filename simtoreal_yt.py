import tkinter as tk
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
ROTATE_SPEED = 50
MOVE_SPEED = 50
WHEEL_DISTANCE = 5.0

# GUI 설정
CELL_PIXELS = 90
MARGIN = 30

SHOW_REAL_BOXES = True          # 실제 박스 위치를 희미하게 보여줄지
DELAY_MOVE = 0.35               # 이동 애니메이션 속도
DELAY_SCAN_STEP = 0.01          # 스캔 애니메이션 속도
DELAY_AFTER_DETECTION = 0.15

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

def cm_to_canvas(x_cm, y_cm):
    x = MARGIN + x_cm / CELL_SIZE_CM * CELL_PIXELS
    y = MARGIN + y_cm / CELL_SIZE_CM * CELL_PIXELS
    return x, y

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
    def __init__(self, box_cells=None):
        if box_cells is None:
            candidates = [(x, y) for x in range(GRID_SIZE_X) for y in range(GRID_SIZE_Y)]
            if (0, 0) in candidates:
                candidates.remove((0, 0))
            random.shuffle(candidates)
            self.box_cells = candidates[:2]
        else:
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
# GUI 시뮬레이터
# ==========================================
class RobotSimulator:
    def __init__(self, root, env=None):
        self.root = root
        self.root.title("새 알고리즘 sim-to-real GUI")

        width = GRID_SIZE_X * CELL_PIXELS + MARGIN * 2
        height = GRID_SIZE_Y * CELL_PIXELS + MARGIN * 2 + 80
        self.canvas = tk.Canvas(root, width=width, height=height, bg="white")
        self.canvas.pack()

        self.env = env if env is not None else SimEnvironment()

        # 시작 위치: (0,0) 셀 중심
        self.robot_x = CELL_SIZE_CM / 2.0
        self.robot_y = CELL_SIZE_CM / 2.0
        self.current_rad = 0.0   # +y 방향

        self.box_coordinates = []
        self.box_count = 0

        self.trace = [(self.robot_x, self.robot_y)]
        self.scan_rays = []

        self.move_count = 0
        self.turn_count_90 = 0.0
        self.scan_count = 0
        self.distance_read_count = 0

        self.status_text = ""

        self.draw_all()
        self.root.after(800, self.run_algorithm)

    # --------------------------
    # 기본 동작
    # --------------------------
    def move_one_cell(self, direction=1):
        self.robot_x += CELL_SIZE_CM * math.sin(self.current_rad) * direction
        self.robot_y += CELL_SIZE_CM * math.cos(self.current_rad) * direction
        self.trace.append((self.robot_x, self.robot_y))
        self.move_count += 1
        self.set_status(f"move_one_cell(direction={direction})")
        self.draw_all()
        time.sleep(DELAY_MOVE)

    def turn(self, angle_rad):
        self.current_rad = normalize_angle(self.current_rad + angle_rad)
        self.turn_count_90 += abs(angle_rad) / (math.pi / 2.0)

    def turn_right(self):
        self.set_status("turn_right()")
        self.turn(math.pi / 2.0)
        self.draw_all()
        time.sleep(DELAY_MOVE)

    def turn_left(self):
        self.set_status("turn_left()")
        self.turn(-math.pi / 2.0)
        self.draw_all()
        time.sleep(DELAY_MOVE)

    def scan(self, end_deg=90):
        self.scan_count += 1
        direction = 1 if end_deg > 0 else -1
        step_count = int(abs(end_deg) / ROTATE_STEP_DEG)

        self.set_status(f"scan({end_deg})")
        for _ in range(step_count):
            self.turn(math.radians(direction * ROTATE_STEP_DEG))

            dist_cm = self.env.raycast_distance(self.robot_x, self.robot_y, self.current_rad)
            self.distance_read_count += 1

            ray_end = None
            if dist_cm is not None and dist_cm <= LIMIT_DISTANCE:
                x = (dist_cm + OFFSET_Y) * math.sin(self.current_rad) + OFFSET_X * math.cos(self.current_rad) + self.robot_x
                y = (dist_cm + OFFSET_Y) * math.cos(self.current_rad) - OFFSET_X * math.sin(self.current_rad) + self.robot_y

                cx = int(x // CELL_SIZE_CM)
                cy = int(y // CELL_SIZE_CM)

                if 0 <= cx < GRID_SIZE_X and 0 <= cy < GRID_SIZE_Y:
                    if (cx, cy) not in self.box_coordinates:
                        self.box_coordinates.append((cx, cy))
                        self.box_count += 1
                        self.set_status(f"박스 감지: {(cx, cy)}")
                        self.draw_all()
                        time.sleep(DELAY_AFTER_DETECTION)

                ray_end = (x, y)
            else:
                ray_end = (
                    self.robot_x + LIMIT_DISTANCE * math.sin(self.current_rad),
                    self.robot_y + LIMIT_DISTANCE * math.cos(self.current_rad),
                )

            self.scan_rays.append(((self.robot_x, self.robot_y), ray_end, dist_cm is not None))
            if len(self.scan_rays) > 50:
                self.scan_rays.pop(0)

            self.draw_all()
            time.sleep(DELAY_SCAN_STEP)

    # --------------------------
    # 알고리즘 본체
    # --------------------------
    def run_algorithm(self):
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

        self.set_status("알고리즘 종료")
        self.draw_all()

        print("=" * 50)
        print("GUI 시뮬레이션 완료")
        print("=" * 50)
        print("실제 박스 좌표:", sorted(self.env.box_cells))
        print("추정 박스 좌표:", sorted(self.box_coordinates))
        print(f"이동 횟수: {self.move_count}")
        print(f"90도 회전 환산 횟수: {self.turn_count_90:.2f}")
        print(f"스캔 호출 횟수: {self.scan_count}")
        print(f"거리센서 측정 횟수: {self.distance_read_count}")

    # --------------------------
    # 그리기
    # --------------------------
    def set_status(self, text):
        self.status_text = text

    def draw_all(self):
        self.canvas.delete("all")

        # 그리드
        for cy in range(GRID_SIZE_Y):
            for cx in range(GRID_SIZE_X):
                x1 = MARGIN + cx * CELL_PIXELS
                y1 = MARGIN + cy * CELL_PIXELS
                x2 = x1 + CELL_PIXELS
                y2 = y1 + CELL_PIXELS

                fill = "white"
                if SHOW_REAL_BOXES and (cx, cy) in self.env.box_cells:
                    fill = "#eeeeee"

                self.canvas.create_rectangle(x1, y1, x2, y2, fill=fill, outline="gray")

                self.canvas.create_text(
                    x1 + 10, y1 + 10,
                    text=f"({cx},{cy})",
                    anchor="nw",
                    fill="gray40",
                    font=("Arial", 9)
                )

        # 실제 박스
        if SHOW_REAL_BOXES:
            for cx, cy in self.env.box_cells:
                center_x_cm, center_y_cm = cell_center_cm(cx, cy)
                px, py = cm_to_canvas(center_x_cm, center_y_cm)
                half = (BOX_SIZE_CM / CELL_SIZE_CM * CELL_PIXELS) / 2.0
                self.canvas.create_rectangle(
                    px - half, py - half, px + half, py + half,
                    fill="#d9d9d9", outline="#999999"
                )

        # 감지된 박스
        for cx, cy in self.box_coordinates:
            center_x_cm, center_y_cm = cell_center_cm(cx, cy)
            px, py = cm_to_canvas(center_x_cm, center_y_cm)
            half = (BOX_SIZE_CM / CELL_SIZE_CM * CELL_PIXELS) / 2.0
            self.canvas.create_rectangle(
                px - half, py - half, px + half, py + half,
                fill="#ff7fbf", outline="#cc0066", width=2
            )

        # 스캔 광선
        for (sx, sy), (ex, ey), hit in self.scan_rays:
            x1, y1 = cm_to_canvas(sx, sy)
            x2, y2 = cm_to_canvas(ex, ey)
            color = "orange" if hit else "#cfe8ff"
            self.canvas.create_line(x1, y1, x2, y2, fill=color)

        # 이동 궤적
        if len(self.trace) >= 2:
            pts = []
            for tx, ty in self.trace:
                px, py = cm_to_canvas(tx, ty)
                pts.extend([px, py])
            self.canvas.create_line(pts, fill="red", width=3, dash=(4, 2))

        # 로봇
        rx, ry = cm_to_canvas(self.robot_x, self.robot_y)
        r = 18
        self.canvas.create_oval(rx - r, ry - r, rx + r, ry + r, fill="#4285F4", outline="black")

        # 방향선
        hx = rx + 28 * math.sin(self.current_rad)
        hy = ry + 28 * math.cos(self.current_rad)
        self.canvas.create_line(rx, ry, hx, hy, fill="yellow", width=4)

        # 상태 텍스트
        info_y = GRID_SIZE_Y * CELL_PIXELS + MARGIN + 10
        self.canvas.create_text(
            MARGIN, info_y,
            anchor="nw",
            text=(
                f"상태: {self.status_text}\n"
                f"실제 박스: {sorted(self.env.box_cells)}\n"
                f"추정 박스: {sorted(self.box_coordinates)}\n"
                f"move={self.move_count}, turn90={self.turn_count_90:.2f}, "
                f"scan={self.scan_count}, dist_reads={self.distance_read_count}"
            ),
            font=("Arial", 11),
            fill="black"
        )

        self.root.update()

# ==========================================
# 실행부
# ==========================================
if __name__ == "__main__":
    root = tk.Tk()

    # 예시:
    env = SimEnvironment(box_cells=[(3, 2), (5, 3)])
    # env = SimEnvironment()

    sim = RobotSimulator(root, env)
    root.mainloop()