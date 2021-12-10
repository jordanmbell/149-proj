import numpy as np

# Configuration Variables
cmd_straight = 0
cmd_clockwise = 1
cmd_anticlockwise = 2

# Parameters to send to robots
class trace_data_t:
    def __init__(self):
        self.cmds = []           # List of ints corresonding to straight, turn, etc.
        self.cmd_param_dist = [] # List of either distances or turning radiuses
        self.cmd_param_angle= [] # List of either 0.'s or turning angles
        self.cmd_times = []      # List of execution times for each cmd

    # Update trace data corresponding to going straight between 2 points
    # that must execute in some give time
    def straight_trace(self, distance, time):
        self.cmds.append(cmd_straight)
        self.cmd_param_dist.append(distance)
        self.cmd_param_angle.append(0.)
        self.cmd_times.append(time)

    # Update trace data corresponding to turning right by <angle> degrees
    # around turning radius <radius>
    # must execute in some time
    def turn_right(self, angle, radius, time):
        self.cmds.append(cmd_clockwise)
        self.cmd_param_dist.append(radius)
        self.cmd_param_angle.append(angle)
        self.cmd_times.append(time)

    # Update trace data corresponding to turning left by <angle> degrees
    # must execute in some time
    def turn_left(self, angle, radius, time):
        self.cmds.append(cmd_anticlockwise)
        self.cmd_param_dist.append(radius)
        self.cmd_param_angle.append(angle)
        self.cmd_times.append(time)


    # Given a list of locations, update the trace to visit all locations
    # Assume we are driving on a grid with grid length = turning radius
    def route(self, locations, turn_radius, speed, angular_speed, setup_time):
        for i in range(len(locations)-1):
            loc1 = locations[i]
            loc2 = locations[i+1]
            if np.abs(loc1[0] - loc2[0]) < turn_radius:  # if no diff in x, go straight
                distance = loc2[1]-loc1[1]
                time = distance / speed
                self.straight_trace(distance, time)
            elif loc2[0] - loc1[0] >= turn_radius:   # If x distance positive, go right
                #First go straight if you can
                distance = np.abs(loc2[1] - loc1[1])
                time = distance / speed
                if distance >= turn_radius:
                    self.straight_trace(distance, speed)

                # Then turn if you can
                distance = np.abs(loc2[0] - loc1[0])
                if distance - turn_radius > 0:
                    distance -= turn_radius
                    turn_time = 90/angular_speed + setup_time
                    self.turn_right(90, turn_radius, turn_time)

                # Then go straight if you can
                time = distance / speed
                if distance > 0:
                    self.straight_trace(distance, time)
            elif loc1[0] - loc2[0] >= turn_radius:   # If x distance negative, go right
                #First go straight if you can
                distance = np.abs(loc2[1] - loc1[1])
                distance -= turn_radius
                time = distance / speed
                if distance > 0:
                    self.straight_trace(distance, speed)

                # Then turn if you can
                distance = np.abs(loc2[0] - loc1[0])
                if distance - turn_radius > 0:
                    distance -= turn_radius
                    turn_time = 90/angular_speed + setup_time
                    self.turn_left(90, turn_radius, turn_time)

                # Then go straight if you can
                time = distance / speed
                if distance > 0:
                    self.straight_trace(distance, time)

    def num_cmds(self):
        return len(self.cmds)

    def __str__(self):
        string = "Trace data\n"
        string += "Cmds: " + str(self.cmds) + "\n"
        string += "Cmd Params Dists: " + str(self.cmd_param_dist) + "\n"
        string += "Cmd Params Angles: " + str(self.cmd_param_angle) + "\n"
        string += "Cmd Times: " + str(self.cmd_times) + "\n"
        return string

if __name__ == "__main__":
    locations = [[0,0], [0, 1], [2,1], [2, 2], [3,3]]

    turn_radius = 0.5  # meter
    speed = 0.01         # meter/sec
    angular_speed = 10  # deg /sec
    setup_time = 4
    trace_data = trace_data_t()
    trace_data.route(locations, turn_radius, speed, angular_speed, setup_time)
    print(trace_data)
