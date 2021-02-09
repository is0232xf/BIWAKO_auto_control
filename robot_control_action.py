
from const import parameter

const = parameter()

def P_control(distance):
    MAX_PULSE = const.MAX_PULSE
    Kp = const.distance_Kp

    t_out = int(1500 + Kp * distance)

    if t_out > MAX_PULSE:
        t_out = MAX_PULSE

    return t_out

def PD_heading_control(diff_deg, e):
    # diff_deg; type: float, unit: [deg]
    # e; type: object(list), unit:[deg]
    Kp = const.degree_Kp
    Kd = const.degree_Kd

    MAX_PULSE = const.MAX_PULSE
    MIN_PULSE = const.MIN_PULSE

    ch = 4

    diff_e = e[len(e)-2]-e[len(e)-1]

    t_out = int(1500 + (Kp * diff_deg + Kd * diff_e * (1/0.02)))

    if t_out < MIN_PULSE:
        t_out = MIN_PULSE
    elif t_out > MAX_PULSE:
        t_out = MAX_PULSE

    action = [ch, t_out]
    return action

def stay_action():
    ch = 4
    pwm = 1500
    action = [ch, pwm]

    if const.debug_mode is True:
        print("Keep the position")
        print("########################")
    return action

def omni_control_action(diff_deg, diff_distance):
    pwm = P_control(diff_distance)
    cmd = 0
    if -45.0 <= diff_deg < 45:
        ch = 5
        cmd = 1

    elif -180.0 <= diff_deg < -135.0 or 135.0 <= diff_deg < 180.0:
        ch = 5
        pwm = 3000 - pwm
        cmd = 2

    elif 45.0 <= diff_deg < 135.0:
        ch = 6
        cmd = 3

    elif -135.0 <= diff_deg < -45.0:
        ch = 6
        pwm = 3000 - pwm
        cmd = 4

    action = [ch, pwm]

    if const.debug_mode is True:
        if cmd is 1:
            print("FORWARD")
        elif cmd is 2:
            print("BACKWARD")
        elif cmd is 3:
            print("RIGHT")
        elif cmd is 4:
            print("LEFT")

        print("pwm: ", pwm)
        print("diff deg: ", diff_deg)
        print("diff distance: ", diff_distance)

    return action

def diagonal_action(diff_deg, diff_distance, deg_e):
    MAX_PULSE = const.MAX_PULSE
    MIN_PULSE = const.MIN_PULSE
    heading_torelance = const.heading_torelance

    cmd = 0

    dir_list = [i*1.0 for i in range(-180, 180+1, 45)]
    # [0]:-180.0, [1]:-135.0, [2]:-90.0, [3]:-45.0,
    # [4]:0.0, [5]:45.0, [6]:90.0, [7]:135.0, [8]:180.0
    if dir_list[5]-heading_torelance < diff_deg < dir_list[5]+heading_torelance:
        # forward first quadrant
        action_1 = [5, 1679]
        action_2 = [6, 1641]
        action = [action_1, action_2]
        cmd = 3
    elif dir_list[3]-heading_torelance < diff_deg < dir_list[3]+heading_torelance:
        # forward second quadrant
        action_1 = [5, 1681]
        action_2 = [6, 1666]
        action = [action_1, action_2]
        cmd = 4
    elif dir_list[1]-heading_torelance < diff_deg < dir_list[1]+heading_torelance:
        # forward third quadrant
        action_1 = [5, 1348]
        action_2 = [6, 1334]
        action = [action_1, action_2]
        cmd = 5
    elif dir_list[7]-heading_torelance < diff_deg < dir_list[7]+heading_torelance:
        # forward fourth quadrant
        action_1 = [5, 1333]
        action_2 = [6, 1643]
        action = [action_1, action_2]
        cmd = 6
    else:
        control_value = PD_heading_control(diff_deg, deg_e)
        action = [control_value, control_value]
        if action[0][1] is 1500:
            cmd = 0
        elif action[0][1] < 1500:
            cmd = 1
        elif action[0][1] > 1500:
            cmd = 2

    if const.debug_mode is True:
        if cmd is 0:
            print('Stop')
        elif cmd is 1:
            print('CW')
        elif cmd is 2:
            print('CCW')
        elif cmd is 3:
            print('First Quadrant')
        elif cmd is 4:
            print('Second Quadrant')
        elif cmd is 5:
            print('Third Quadrant')
        elif cmd is 6:
            print('Fourth Quadrant')

        print("action: ", action)
        print("diff deg: ", diff_deg)
        print("diff distance: ", diff_distance)

    return action
