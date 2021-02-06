
from const import parameter

const = parameter()

def P_control(distance):
    MAX_PULSE = const.MAX_PULSE
    Kp = const.distance_Kp

    t_out = int(1500 + Kp * distance)

    if t_out > MAX_PULSE:
        t_out = MAX_PULSE

    return t_out

def stay_action():
    ch = 4
    pwm = 1500
    action = [ch, pwm]

    if const.debug_mode == True:
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

    if const.debug_mode == True:
        if cmd == 1:
            print("FORWARD)
        elif cmd == 2:
            print("BACKWARD")
        elif cmd == 3:
            print("RIGHT")
        elif cmd == 4:
            print("LEFT")

        print("pwm: ", pwm)
        print("diff deg: ", diff_deg)
        print("diff distance: ", diff_distance)

    return action
