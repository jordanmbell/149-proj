import datetime
from discrete_signals import signal
import math


# Global variables: Specific for ros Odemetry, IMU, and Bumper
## Position
x_pos_raw = []
y_pos_raw = []
z_pos_raw = []
### Signals
x_pos = signal([], start=0, end=float('inf'), tag='x_pos')
y_pos = signal([], start=0, end=float('inf'), tag='y_pos')
z_pos = signal([], start=0, end=float('inf'), tag='z_pos')


## Orientation
x_ori_raw = []
y_ori_raw = []
z_ori_raw = [] 
w_ori_raw = []
### Signals
x_ori = signal([], start=0, end=float('inf'), tag='x_ori')
y_ori = signal([], start=0, end=float('inf'), tag='y_ori')
z_ori = signal([], start=0, end=float('inf'), tag='z_ori')
w_ori = signal([], start=0, end=float('inf'), tag='w_ori')


## Linear twist
x_twist_raw = []
y_twist_raw = []
z_twist_raw = []
### Signals
x_twist = signal([], start=0, end=float('inf'), tag='x_twist')
y_twist = signal([], start=0, end=float('inf'), tag='y_twist')
z_twist = signal([], start=0, end=float('inf'), tag='z_twist')


## Angular twist
x_ang_twist_raw = []
y_ang_twist_raw = []
z_ang_twist_raw = []
### Signals
x_ang_twist = signal([], start=0, end=float('inf'), tag='x_ang_twist')
y_ang_twist = signal([], start=0, end=float('inf'), tag='y_ang_twist')
z_ang_twist = signal([], start=0, end=float('inf'), tag='z_ang_twist')

## IMU orientation 
imu_x_ori_raw = []
imu_y_ori_raw = []
imu_z_ori_raw = []
imu_w_ori_raw = []
### Signals
imu_x_ori = signal([], start=0, end=float('inf'), tag='imu_x_ori')
imu_y_ori = signal([], start=0, end=float('inf'), tag='imu_y_ori')
imu_z_ori = signal([], start=0, end=float('inf'), tag='imu_z_ori')
imu_w_ori = signal([], start=0, end=float('inf'), tag='imu_w_ori')

## Angular velocity
x_ang_vel_raw = []
y_ang_vel_raw = []
z_ang_vel_raw = []
### Signals
x_ang_vel = signal([], start=0, end=float('inf'), tag='x_ang_vel')
y_ang_vel = signal([], start=0, end=float('inf'), tag='y_ang_vel')
z_ang_vel = signal([], start=0, end=float('inf'), tag='z_ang_vel')

## Linear acceleration 
x_acc_raw = []
y_acc_raw = []
z_acc_raw = []
### Signals
x_acc = signal([], start=0, end=float('inf'), tag='x_acc')
y_acc = signal([], start=0, end=float('inf'), tag='y_acc')
z_acc = signal([], start=0, end=float('inf'), tag='z_acc')

## Angle
x_angle_raw = []
y_angle_raw = []
z_angle_raw = []
### Signals
x_angle = signal([], start=0, end=float('inf'), tag='x_angle')
y_angle = signal([], start=0, end=float('inf'), tag='y_angle')
z_angle = signal([], start=0, end=float('inf'), tag='z_angle')

## Bumper 
bump_raw = [(0.0,-1)]
bump = signal([], start=0, end=float('inf'), tag='bump')

## Cliff
cliff_raw = [(0.0,-1)]
cliff = bump = signal([], start=0, end=float('inf'), tag='cliff')


# Parse Utils
## Check if current file line is a data line
def data_line(line):

    return ("Odometry:" in line) or ("IMU Data:" in line) or ("Cliff Event:" in line) or ("Bumper Event:" in line) or ("Angle:" in line)

def parse_ros_line(line, time0, last_odometry_time, last_imu_time, last_angle_time, last_bump_time, last_cliff_time, sample_rate, log_type):

    global x_pos, y_pos, z_pos, x_ori, y_ori, z_ori, w_ori, x_twist, y_twist, z_twist, x_ang_twist, y_ang_twist, z_ang_twist, imu_x_ori, imu_y_ori, imu_z_ori, imu_w_ori, x_ang_vel, y_ang_vel, z_ang_vel, x_acc, y_acc, z_acc, x_angle, y_angle, z_angle, bump, cliff_raw
    
    if log_type=="cpp":
        tokens = line.split(" ",1)
    else:
        tokens = line.split(" ",3)

    if log_type=="cpp":
        #convert sec to min. Necessary for datetime type
        time = f"{math.floor(float(tokens[0][:-6])/60)}:{round(float(tokens[0][:6])%60,3)}"
        time_raw = datetime.datetime.strptime(time,"%M:%S.%f")
    else:
        time_raw = datetime.datetime.strptime(tokens[2][:-1],"%H:%M:%S,%f")    
    # Retrieve relative time
    
    
    time_temp = time_raw - datetime.datetime(1900, 1, 1)
    time_stamp = round(time_temp.total_seconds()-time0,3)
    
    

    # Data
    position_values = []
    orientation_values = []
    twist_values = []
    ang_twist_values = []
    imu_orientation_values = []
    ang_velocity_values = []
    linear_acceleration_values = []
    bumper_value = []
    cliff_value = []

    # Retrieve Measuring unit 
    if "Odometry:" in line:
        unit="Odometry"
    elif "IMU Data:" in line:
        unit="IMU Data"
    elif "Bumper Event:" in line:
        unit="Bumper Event"
    elif "Cliff Event:" in line:
        unit="Cliff Event"
    elif "Angle:" in line:
        unit="Angle"
    else:
        raise SyntaxError

    # Retrieve odomentry data
    if (unit=="Odometry" and (((time_stamp - last_odometry_time)>=sample_rate ) or (time_stamp==0.0))):
        # position
        tokens = tokens[-1].split("=",1) 
        tokens = tokens[-1].split("(",1)
        tokens = tokens[-1].split("),",1)
        position_values = tokens[0].split(", ")
        x_pos_raw.append((time_stamp, float(position_values[0])))
        y_pos_raw.append((time_stamp, float(position_values[1])))
        z_pos_raw.append((time_stamp, float(position_values[2])))
        # orientation
        tokens = tokens[-1].split("=",1) 
        tokens = tokens[-1].split("(",1)
        tokens = tokens[-1].split("),",1)
        orientation_values = tokens[0].split(", ")
        x_ori_raw.append((time_stamp, float(orientation_values[0])))
        y_ori_raw.append((time_stamp, float(orientation_values[1])))
        z_ori_raw.append((time_stamp, float(orientation_values[2])))
        w_ori_raw.append((time_stamp, float(orientation_values[3])))
        # linear twist
        tokens = tokens[-1].split("=",1) 
        tokens = tokens[-1].split("(",1)
        tokens = tokens[-1].split("),",1)
        twist_values = tokens[0].split(", ")
        x_twist_raw.append((time_stamp, float(twist_values[0])))
        y_twist_raw.append((time_stamp, float(twist_values[1])))
        z_twist_raw.append((time_stamp, float(twist_values[2])))
        # angular twist
        tokens = tokens[-1].split("=",1) 
        tokens = tokens[-1].split("(",1)
        tokens = tokens[-1].split(")",1)
        ang_twist_values = tokens[0].split(", ")
        x_ang_twist_raw.append((time_stamp, float(ang_twist_values[0])))
        y_ang_twist_raw.append((time_stamp, float(ang_twist_values[1])))
        z_ang_twist_raw.append((time_stamp, float(ang_twist_values[2])))
        #no bump
        if (last_bump_time < time_stamp):
            bumper_value = -1.0
            bump_raw.append((time_stamp,bumper_value))
            last_bump_time = time_stamp
        if (last_cliff_time < time_stamp):
            cliff_value = -1.0
            cliff_raw.append((time_stamp,cliff_value))
            last_cliff_time = time_stamp
        
        return 1, time_stamp, last_imu_time, last_angle_time, last_bump_time, last_cliff_time

    # Retrieve IMU data 
    if (unit=="IMU Data" and (((time_stamp - last_imu_time)>=sample_rate ) or (time_stamp==0.0))):
        # orientation
        tokens = tokens[-1].split("=",1) 
        tokens = tokens[-1].split("(",1)
        tokens = tokens[-1].split("),",1)
        imu_orientation_values = tokens[0].split(", ")
        imu_x_ori_raw.append((time_stamp, float(imu_orientation_values[0])))
        imu_y_ori_raw.append((time_stamp, float(imu_orientation_values[1])))
        imu_z_ori_raw.append((time_stamp, float(imu_orientation_values[2])))
        imu_w_ori_raw.append((time_stamp, float(imu_orientation_values[3])))
        # angular velocity
        tokens = tokens[-1].split("=",1) 
        tokens = tokens[-1].split("(",1)
        tokens = tokens[-1].split("),",1)
        ang_velocity_values = tokens[0].split(", ")
        x_ang_vel_raw.append((time_stamp, float(ang_velocity_values[0])))
        y_ang_vel_raw.append((time_stamp, float(ang_velocity_values[1])))
        z_ang_vel_raw.append((time_stamp, float(ang_velocity_values[2])))
        # linear acceleration 
        tokens = tokens[-1].split("=",1) 
        tokens = tokens[-1].split("(",1)
        tokens = tokens[-1].split(")",1)
        linear_acceleration_values = tokens[0].split(", ")
        x_acc_raw.append((time_stamp, float(linear_acceleration_values[0])))
        y_acc_raw.append((time_stamp, float(linear_acceleration_values[1])))
        z_acc_raw.append((time_stamp, float(linear_acceleration_values[2])))
        # no bump and no cliff
        if (last_bump_time < time_stamp):
            bumper_value = -1.0
            bump_raw.append((time_stamp,bumper_value))
            last_bump_time = time_stamp
        if (last_cliff_time < time_stamp):
            cliff_value = -1.0
            cliff_raw.append((time_stamp,cliff_value))
            last_cliff_time = time_stamp

        return 1, last_odometry_time, time_stamp, last_angle_time, last_bump_time, last_cliff_time

        # print(time_stamp, unit, imu_orientation_values, ang_velocity_values)

    if (unit=="Angle"):
        tokens = tokens[-1].split("=",1) 
        tokens = tokens[-1].split("(",1)
        tokens = tokens[-1].split(")",1)
        angle_values = tokens[0].split(", ")
        x_angle_raw.append((time_stamp,int(float(angle_values[0])))) 
        y_angle_raw.append((time_stamp,int(float(angle_values[1]))))
        z_angle_raw.append((time_stamp,int(float(angle_values[2]))))            
        # no bump and no cliff
        if (last_bump_time < time_stamp):
            bumper_value = -1.0
            bump_raw.append((time_stamp,bumper_value))
            last_bump_time = time_stamp
        if (last_cliff_time < time_stamp):
            cliff_value = -1.0
            cliff_raw.append((time_stamp,cliff_value))
            last_cliff_time = time_stamp
        
        return 1, last_odometry_time, last_imu_time, time_stamp, last_bump_time, last_cliff_time


    # Retrive bumper data
    if (unit=="Bumper Event"):
        tokens = tokens[-1].split("Bumper Event:",1) 
        bumper_value = float(tokens[1])
        bump_raw.append((time_stamp,bumper_value))

        # print(time_stamp, unit, bumper_value)

        return 1, last_odometry_time,last_imu_time, last_angle_time, time_stamp, last_cliff_time

     # Retrive cliff data
    if (unit=="Cliff Event"):
        tokens = tokens[-1].split("Cliff Event:",1) 
        cliff_value = float(tokens[1])
        cliff_raw.append((time_stamp,cliff_value))

        # print(time_stamp, unit, cliff_value)

        return 1, last_odometry_time,last_imu_time, last_angle_time, last_bump_time, time_stamp

    


    return 0, last_odometry_time, last_imu_time, last_angle_time, last_bump_time, last_cliff_time


def retrieve_data(path, sample_rate, log_type):
    file = open(path,"r")
    count = 0
    first_ts = True
    time0 = 0.0
    last_odometry_time = 0.0
    last_imu_time = 0.0
    last_angle_time = 0.0
    last_bump_time =0.0
    last_cliff_time = 0.0
    time_temp = 0.0

    for line in file:
        if data_line(line):
            if(first_ts):
                if log_type =="cpp":
                    tokens = line.split(" ",1)
                    time_raw = datetime.datetime.strptime(tokens[0][:-6],"%S.%f")
                else:
                    tokens = line.split(" ",3)
                    time_raw = datetime.datetime.strptime(tokens[2][:-1],"%H:%M:%S,%f")
                time_temp = time_raw - datetime.datetime(1900, 1, 1)
                time0 = time_temp.total_seconds()
                first_ts = False

            inc, last_odometry_time, last_imu_time, last_angle_time, last_bump_time, last_cliff_time = parse_ros_line(line,time0,last_odometry_time,last_imu_time,last_angle_time,last_bump_time,last_cliff_time ,sample_rate,log_type)
            count += inc

            # if count==800:
            #     raise SyntaxError

    return count, time_temp


def create_signals():

    global x_pos, y_pos, z_pos, x_ori, y_ori, z_ori, w_ori, x_twist, y_twist, z_twist, x_ang_twist, y_ang_twist, z_ang_twist, imu_x_ori, imu_y_ori, imu_z_ori, imu_w_ori, x_ang_vel, y_ang_vel, z_ang_vel, x_acc, y_acc, z_acc, x_angle, y_angle, z_angle, bump, cliff

    # position 
    x_pos = signal(x_pos_raw, start=0, end=float('inf'), tag='x_pos')
    y_pos = signal(y_pos_raw, start=0, end=float('inf'), tag='y_pos')
    z_pos = signal(z_pos_raw, start=0, end=float('inf'), tag='z_pos')

    # orientation
    x_ori = signal(x_ori_raw, start=0, end=float('inf'), tag='x_ori')
    y_ori = signal(y_ori_raw, start=0, end=float('inf'), tag='y_ori')
    z_ori = signal(z_ori_raw, start=0, end=float('inf'), tag='z_ori')
    w_ori = signal(w_ori_raw, start=0, end=float('inf'), tag='w_ori')

    # twist
    x_twist = signal(x_twist_raw, start=0, end=float('inf'), tag='x_twist')
    y_twist = signal(y_twist_raw, start=0, end=float('inf'), tag='y_twist')
    z_twist = signal(z_twist_raw, start=0, end=float('inf'), tag='z_twist')     

    # angular twist
    x_ang_twist = signal(x_ang_twist_raw, start=0, end=float('inf'), tag='x_ang_twist')
    y_ang_twist = signal(y_ang_twist_raw, start=0, end=float('inf'), tag='y_ang_twist')
    z_ang_twist = signal(z_ang_twist_raw, start=0, end=float('inf'), tag='z_ang_twist')     

    # imu orientation
    imu_x_ori = signal(imu_x_ori_raw, start=0, end=float('inf'), tag='imu_x_ori')
    imu_y_ori = signal(imu_y_ori_raw, start=0, end=float('inf'), tag='imu_y_ori')
    imu_z_ori = signal(imu_z_ori_raw, start=0, end=float('inf'), tag='imu_z_ori')
    imu_w_ori = signal(imu_w_ori_raw, start=0, end=float('inf'), tag='imu_w_ori')

    # angular velocity
    x_ang_vel = signal(x_ang_vel_raw, start=0, end=float('inf'), tag='x_ang_vel')
    y_ang_vel = signal(y_ang_vel_raw, start=0, end=float('inf'), tag='y_ang_vel')
    z_ang_vel = signal(z_ang_vel_raw, start=0, end=float('inf'), tag='z_ang_vel')

    # acceleration
    x_acc = signal(x_acc_raw, start=0, end=float('inf'), tag='x_acc')
    y_acc = signal(y_acc_raw, start=0, end=float('inf'), tag='y_acc')
    z_acc = signal(z_acc_raw, start=0, end=float('inf'), tag='z_acc')

    # angle
    x_angle = signal(x_angle_raw, start=0, end=float('inf'), tag='x_angle')
    y_angle = signal(y_angle_raw, start=0, end=float('inf'), tag='y_angle')
    z_angle = signal(z_angle_raw, start=0, end=float('inf'), tag='z_angle')

    # bumper
    bump = signal(bump_raw, start=0, end=float('inf'), tag='bump')

    # cliff 
    cliff = signal(cliff_raw,start=0,end=float('inf'), tag='cliff')
   

