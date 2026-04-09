left_color_sensor = 'A'
right_color_sonsor = 'B'

lcolor = ColorSensor(left_color_sensor)
rcolor = ColorSensor(right_color_sensor)

if lcolor.get_color() == 'white' and rcolor.get_color() == 'black':
    while True:
        move_distance(motor_pair, -2)
        rotate_robot(motor_pair, 5)
        move_distance(motor_pair, 2)
        if lcolor.get_color() == 'black' and rcolor.get_color() == 'black':
            break

else if lcolor.get_color() == 'black' and rcolor.get_color() == 'white':
    while True:
        move_distance(motor_pair, -2)
        rotate_robot(motor_pair, -5)
        move_distance(motor_pair, 2)
        if lcolor.get_color() == 'black' and rcolor.get_color() == 'black':
            break