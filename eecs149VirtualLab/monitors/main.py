from utils import signals
import argparse
import monitors



# Parse arguments
parser = argparse.ArgumentParser(description='simple ramp monitors', usage='%(prog)s [-h] log_file_path config_file_path [--sample_rate]', formatter_class=argparse.ArgumentDefaultsHelpFormatter)

## required arguments

parser.add_argument('log_file_path', help='trace file')
parser.add_argument('config_file_path', help='map information')
parser.add_argument('--sample_rate', help='sample rate', type=float, default=0.1)
parser.add_argument('--margin', help='out of bounds margin', type=float, default=0.13)
parser.add_argument('--cliff_margin', help='margin to cliff', type=float, default=0.13)
parser.add_argument('--circle_time', help='time to circle in place once', type=float, default=25.0)
parser.add_argument('--timeout',help='timeout for reaching top and returning to bottom after reaching top',type=float,default=120.0)
parser.add_argument('--robot_radius',help='radius of robot', type=float,default=0.176)
parser.add_argument('--obstacle_radius',help='radius of obstacle', type=float,default=0.27)
parser.add_argument('--backing_off_time', help='time to back off from cliff', type=float,default=5.0)
parser.add_argument('--backing_off_distance', help='distance to back off from obstacle after bump', type=float,default=.1)
parser.add_argument('--rotation_time', help='time to rotate after cliff or bump', type=float,default=20.0)
parser.add_argument('--cliff_turn_angle', help='angle to turn on cliff', type=float,default=5.0)
parser.add_argument('--bump_turn_angle', help='angle to turn on bump', type=float,default=45.0)
parser.add_argument('--initialization_time', help='time to start climbing', type=float,default=25.0)
parser.add_argument('--log_type', help="python is default. if cpp use value cpp", default="python",choices=["python","cpp"])

args = parser.parse_args()

## Map Parameter: 
log_file_path = args.log_file_path
config_file_path = args.config_file_path
sample_rate = args.sample_rate
margin = args.margin
cliff_margin = args.cliff_margin
circle_time = args.circle_time
timeout = args.timeout
robot_radius = args.robot_radius
obstacle_radius = args.obstacle_radius
backing_off_time = args.backing_off_time
backing_off_distance = args.backing_off_distance
initialization_time = args.initialization_time
rotation_time = args.rotation_time
cliff_turn_angle = args.cliff_turn_angle
bump_turn_angle = args.bump_turn_angle
log_type = args.log_type

## Parse config file
ramp_coordinates = []
ramp_coordinates_strings = []
obstacles_positions = []
obstacles_strings = []

config_file = open(config_file_path,"r")
for line in config_file.readlines():
    tokens = line.split(":")
    if tokens[0] == "ramp":
        ramp_coordinates_strings = tokens[1].strip().strip("[").rstrip("\n").rstrip("]").split(";")
    elif tokens[0] == "obstacle":
        obstacles_strings = tokens[1].strip().strip("[").rstrip("\n").rstrip("]").split(";")
    else:
        raise SyntaxError(f"Unknown keyword '{tokens[0]}'!")

for s in ramp_coordinates_strings:
    tokens = s.strip("(").rstrip(")").split(",")
    ramp_coordinates.append((float(tokens[0]),float(tokens[1])))

for s in obstacles_strings:
    tokens = s.strip("(").rstrip(")").split(",")
    obstacles_positions.append((float(tokens[0]),float(tokens[1])))

print(f"Ramp coordinates: {ramp_coordinates}")
print(f"Obstacles coordinates: {obstacles_positions}")


# retrieve raw data from log file
trace_length = 0
time0 = 0.0
trace_length, time0 = signals.retrieve_data(log_file_path, sample_rate,log_type)

print("Trace length:",trace_length,"events")
## Initialize signals 
signals.create_signals()

# ====================================================================================================
# ==================================== Execute Monitors ==============================================
if ramp_coordinates!=[]:
    print("Executing ramp monitors...")
    monitors.out_of_bounds(ramp_coordinates,margin,time0)
    monitors.circling(circle_time,initialization_time,time0)
    monitors.reached_top(ramp_coordinates,0,robot_radius,time0,backing_off_time,initialization_time,timeout)
    monitors.return_to_bottom(ramp_coordinates,robot_radius,time0,timeout)
    monitors.rotate_on_cliff(ramp_coordinates,robot_radius,rotation_time,cliff_turn_angle,time0)
    monitors.back_off_on_cliff(ramp_coordinates, backing_off_time, cliff_margin,time0)

if obstacles_positions!=[]:
    print("Executing obstacle monitors...")
    monitors.rotate_on_bump(rotation_time,bump_turn_angle,time0)
    monitors.back_off_on_bump(obstacles_positions,obstacle_radius,robot_radius,backing_off_time,backing_off_distance,time0,timeout)
    monitors.evade_obstacle(obstacles_positions,obstacle_radius, robot_radius, backing_off_distance, bump_turn_angle, rotation_time,backing_off_time,time0,timeout)
    

# ======================================== End Execute Monitor ===================================== #
# ================================================================================================== #
######################################################################################################










