# import library
import time
import mpu
import csv
import datetime
import math
import signal
import threading

import numpy as np
from const import parameter
from pymavlink import mavutil
from geopy.distance import geodesic
from robot import Robot
from wt_sensor_class import WT_sensor
import calculate_degree as calculator
from INA226 import INA226

const = parameter()

# read waypoint file (csv)
target_point_file = const.way_point_file
target_point = np.genfromtxt(target_point_file,
                          delimiter=',',
                          dtype='float',
                          encoding='utf-8')

# connect to the robot
master = mavutil.mavlink_connection('udp:127.0.0.01:14551')

# wait heart beat
master.wait_heartbeat()

# Request all parameters
master.mav.param_request_list_send(
    master.target_system, master.target_component
)

# set robot mode to manual and armable
def set_mode(mode):
    # Get mode ID: return value should be 1 for acro mode
    mode_id = master.mode_mapping()[mode]
    # Set new mode
    master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

def set_arm_disarm(msg):
    if(msg=='ARM'):
        cmd = 1
    elif(msg=='DISARM'):
        cmd = 0
    ############################################################## Arm
    master.mav.command_long_send(master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    cmd,
    1, 0, 0, 0, 0, 0, 0)

# update the robot state
def update_robot_state():
    GPS_message = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()
    attitude_message = master.recv_match(type='ATTITUDE', blocking=True).to_dict()
    lon = float(GPS_message['lon'])/10**7
    lat = float(GPS_message['lat'])/10**7
    yaw = float(attitude_message['yaw'])

    BIWAKO.lon = lon
    BIWAKO.lat = lat
    BIWAKO.yaw = yaw

# control thrusters
def control_thruster(action, ch=0, pwm=1500):
    ch = action[0]
    pwm = action[1]
    if ch < 1:
        print("Channel does not exist.")
    if ch < 9:
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[ch - 1] = pwm
        master.mav.rc_channels_override_send(
            master.target_system,                # target_system
            master.target_component,             # target_component
            *rc_channel_values)                  # RC channel list, in microseconds.

# calculate heading difference between current point to target point
def calc_heading_diff(way_point):
    t_lon = math.radians(way_point[0])
    t_lat = math.radians(way_point[1])
    c_lon = math.radians(BIWAKO.lon)
    c_lat = math.radians(BIWAKO.lat)
    d_lon = c_lon - t_lon

    diff_heading = 90 - math.degree(math.atan2(math.cos(t_lat)*math.sin(c_lat)
                  - math.sin(t_lat)*math.cos(c_lat)*math.cos(d_lon), math.sin(d_lon)*math.cos(c_lat)))
    return diff_heading

def decide_next_action(pose, start_time, distace_tolerance, heading_torelance):
    # decide the next action from current robot status and the next waypoint
    current_point = np.array([pose[1], pose[0]])
    current_yaw = pose[2]

    diff_distance = round(mpu.haversine_distance(current_point, BIWAKO.next_goal), 5)*1000
    # check distance between current and target
    if abs(diff_distance) < distace_tolerance:
        action = stay_action()
    # when the device has not received
    else:
        target_direction = math.radians(calculator.calculate_bearing(current_point, BIWAKO.next_goal))
        diff_deg =  math.degrees(calculator.limit_angle(target_direction - current_yaw))
        action = omni_control_action()
    return action

def P_control(distance):
    MAX_PULSE = const.MAX_PULSE
    Kp = const.distance_Kp

    t_out = int(1500 + Kp * distance)

    if t_out > MAX_PULSE:
        t_out = MAX_PULSE

    return t_out

def kill_signal_process(arg1, args2):
    pass

def logging(arg1, args2):
    update_robot_state()
    wt = wt_sensor.wt
    v = power_sensor.get_voltage()
    c = power_sensor.get_current()
    p = power_sensor.get_power()
    unix_time = time.time()
    BIWAKO.count = BIWAKO.count + 0.1
    data = [unix_time, BIWAKO.count, BIWAKO.lat, BIWAKO.lon, math.degrees(BIWAKO.yaw),
            BIWAKO.cmd, BIWAKO.pwm, wt, v, c, p]
    log_data.append(data)

def calc_temp_target(current_point, target_point):
    current_point = np.array([current_point])
    target_point = np.array([target_point])
    temp_target = target_point-(current_point-target_point)/2
    temp_target = [temp_target[0][0], temp_target[0][1]]
    return temp_target

def omni_control_action(diff_deg, diff_distance):
    pwm = P_control(diff_distance)
    if -45.0 <= diff_deg < 45:
        ch = 5
        print("Forward")

    elif -180.0 <= diff_deg < -135.0 or 135.0 <= diff_deg < 180.0:
        ch = 5
        pwm = 3000 - pwm
        print("Backward")

    elif 45.0 <= diff_deg < 135.0:
        ch = 6
        print("Right")

    elif -135.0 <= diff_deg < -45.0:
        ch = 6
        pwm = 3000 - pwm
        print("Left")

    action = [ch, pwm]
    print("pwm: ", pwm)

    print("diff deg: ", diff_deg)
    print("diff distance: ", diff_distance)
    return action

def stay_action():
    ch = 4
    pwm = 1500
    action = [ch, pwm]

    print("Keep the position")
    print("########################")
    return action

def update_wt():
    while True:
        wt_sensor.wt = wt_sensor.observation()

###############################################################################
# Initialize the robot
set_mode('MANUAL')
print("Set mode to Manual")
initial_action = [4, 1500]
print('Initialize...')
print('Wait seven second...')
control_thruster(initial_action)
time.sleep(7)
master.arducopter_arm()
print("Arm/Disarm: Arm")

BIWAKO = Robot(target_point)
wt_sensor = WT_sensor()
addr = 0x40
power_sensor = INA226(addr)
power_sensor.initial_operation()

update_wt_thread = threading.Thread(target=update_wt)
update_wt_thread.start()

log_data = []

############ start to measure the second ############
start_time = 0.0

if __name__ == '__main__':

    state_data_log = const.data_log_mode
    debug_mode = const.debug_mode

    main_target_distance_torelance = const.main_target_distance_torelance
    temp_target_distance_torelance = const.temp_target_distance_torelance
    heading_torelance = const.heading_torelance
    keep_time = const.duration

    if (state_data_log==True):
        # get date time object
        detail = datetime.datetime.now()
        date = detail.strftime("%Y%m%d%H%M%S")
        # open csv file
        file = open('./csv/'+ date +'.csv', 'a', newline='')
        csvWriter = csv.writer(file)
        csvWriter.writerow(['time', 'count', 'latitude', 'longitude', 'yaw', 'cmd',
                            'pwm', 'wt', 'voltage', 'current', 'power_consumption'])

    try:
        signal.signal(signal.SIGALRM, logging)
        signal.setitimer(signal.ITIMER_REAL, 0.5, 0.5)
        while True:
            pose = [BIWAKO.lon, BIWAKO.lat, BIWAKO.yaw]
            # decide the next action from current robot status and the next waypoint
            current_point = np.array([pose[1], pose[0]])
            current_yaw = pose[2]
            diff_distance = round(mpu.haversine_distance(current_point, BIWAKO.next_goal), 5)*1000

            if abs(diff_distance) < main_target_distance_torelance:
                action = stay_action()
                BIWAKO.cmd = action[0]
                BIWAKO.pwm = action[1]
                control_thruster(action)
                time.sleep(0.02)

            else:
                BIWAKO.temp_goal = calc_temp_target(current_point, BIWAKO.next_goal)
                while True:
                    pose = [BIWAKO.lat, BIWAKO.lon, BIWAKO.yaw]
                    # decide the next action from current robot status and the next waypoint
                    current_point = np.array([pose[0], pose[1]])
                    current_yaw = pose[2]

                    # location input format: [latitude, longotude]
                    diff_distance = round(mpu.haversine_distance(current_point, BIWAKO.temp_goal), 5)*1000
                    target_direction = math.radians(calculator.calculate_bearing(current_point, BIWAKO.temp_goal))
                    diff_deg =  math.degrees(calculator.limit_angle(target_direction - current_yaw))
                    action = omni_control_action(diff_deg, diff_distance)
                    BIWAKO.cmd = action[0]
                    BIWAKO.pwm = action[1]
                    control_thruster(action)
                    time.sleep(0.02)
                    if abs(diff_distance) < temp_target_distance_torelance:
                        break


        if (state_data_log==True):
            for i in range(len(log_data)):
                csvWriter.writerow(log_data[i])
            file.close()

        update_wt_thread.join()
        master.arducopter_disarm()
        print("Arm/Disarm: Disarm")
        signal.signal(signal.SIGALRM, kill_signal_process)
        signal.setitimer(signal.ITIMER_REAL, 1, 0.5)

    except KeyboardInterrupt:
        if (state_data_log==True):
            for i in range(len(log_data)):
                csvWriter.writerow(log_data[i])
            file.close()

        update_wt_thread.join()
        master.arducopter_disarm()
        print("Arm/Disarm: Disarm")
        time.sleep(3)
        signal.signal(signal.SIGALRM, kill_signal_process)
        signal.setitimer(signal.ITIMER_REAL, 1, 0.5)
