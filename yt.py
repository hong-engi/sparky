import math
from spike import MotorPair, Motor, DistanceSensor

left_motor_port = 'A'
right_motor_port = 'B'
distance_sensor_port = 'C'

robot_x = cell_size/2
robot_y = cell_size/2

offset_x = 0
offset_y = 0

current_deg = 0

cell_size = 23
grid_size_x = 6
grid_size_y = 4
box_size = 15

rotate_speed = 50
rotate_step = 0.5

move_speed = 50

box_count = 0
box_coordinates = []
limit_distance = cell_size*6

# 바퀴 사이 간격
wheel_distance = 5

motor_pair = MotorPair(left_motor_port, right_motor_port)
distance = DistanceSensor(distance_sensor_port)

def move_one_cell(direction=1):
    global robot_x, robot_y

    motor_pair.move(cell_size * direction, 'cm', steering=0, speed=move_speed)

    robot_x += cell_size * math.sin(current_deg) * direction
    robot_y += cell_size * math.cos(current_deg) * direction

def turn(angle_rad):
    global current_deg
    direction = 1 if angle_rad > 0 else -1
    arc = abs(angle_rad) * (wheel_distance / 2)

    motor_pair.move(arc, 'cm', steering=100 * direction, speed=rotate_speed)
    current_deg += angle_rad

def turn_right():
    turn(math.pi/2)

def turn_left():
    turn(-math.pi/2)

def scan(end_deg=90):
    global box_coordinates, box_count

    direction = 1 if end_deg > 0 else -1
    step_count = int(abs(end_deg / rotate_step))

    for i in range(step_count):
        turn(math.radians(direction * rotate_step))

        dist_cm = distance.get_distance_cm(short_range=False)
        if dist_cm is None or dist_cm > limit_distance:
            continue
        
        x = (dist_cm + offset_y) * math.sin(current_deg) + offset_x * math.cos(current_deg) + robot_x
        y = (dist_cm + offset_y) * math.cos(current_deg) - offset_x * math.sin(current_deg) + robot_y

        cx = int(x // cell_size)
        cy = int(y // cell_size)

        if (cx, cy) not in box_coordinates:
            box_coordinates.append((cx, cy))
            box_count += 1

scan(90)

if box_count == 1:
    if box_coordinates[0][0] == 0:
        move_one_cell()
        scan(-180)
    else:
        turn_right()
        move_one_cell(direction=-1)
        scan(-90)
        if box_count == 1:
            scan(-90)
