
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from trace import *

# Set up graph
frame_size = 5
plt.style.use('seaborn')
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(1, 1, 1)
ax.set_xlim([-frame_size, frame_size])
ax.set_ylim([-frame_size, frame_size])
colors = ['blue', 'orange', 'green', 'red', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']

# This function is called periodically from FuncAnimation
# xs and ys are lists of past x,y to show a trail
def animate(i, xs, ys, shared_dict, num_markers, trace_data, route_locations, starting_orientation):

    # Add x and y to lists
    #print(shared_dict)
    rot_list = []
    for k in range(num_markers):
        x, y, rot = shared_dict[k]
        xs[k].append(x)
        ys[k].append(y)
        rot_list.append(rot)

    # Limit x and y lists to n_items
    n_items = 20
    xs = xs[-n_items:]
    ys = ys[-n_items:]


    # Draw x and y lists
    ax.clear()
    n_div = 20
    for k in range(num_markers):
        for j in range(n_div):
            x_range, y_range = xs[k][-(n_items - (n_items//n_div)*j):], ys[k][-(n_items - (n_items//n_div)*j):]
            ax.plot(x_range, y_range, alpha=0.1*j, c=colors[k] )

    for k in range(num_markers):
        x, y = xs[k][-1], ys[k][-1]
        ax.scatter(x, y,  c=colors[k], s=20, label=str(k))
        rot = np.radians(rot_list[k]) + np.pi/2
        arrowx, arrowy = np.cos(rot), np.sin(rot)
        ax.arrow(x, y, arrowx*0.3, arrowy*0.3, head_length = 0.1, head_width = 0.1)

    # Plot route_locations
    for l in range(len(route_locations)):
        ax.scatter(route_locations[l][0], route_locations[l][1],  c='black', s=20, label="Target: " + str(l), marker ="*")

    # Plot trace
    n_cmds = len(trace_data.cmds)
    loc = np.array(route_locations[0])
    orientation = starting_orientation
    for l in range(n_cmds):
        next_loc = None
        cmd = trace_data.cmds[l]
        if cmd == cmd_straight:
            distance = trace_data.cmd_param_dist[l]
            next_loc = orientation * distance + loc
        elif cmd == cmd_clockwise:
            turn_radius = trace_data.cmd_param_dist[l]
            angle = -np.radians(trace_data.cmd_param_angle[l])
            rotation = np.array([
                [np.cos(angle), -np.sin(angle)],
                [np.sin(angle), np.cos(angle)]
            ])
            next_loc = orientation * turn_radius
            next_loc += rotation @ orientation * turn_radius + loc
            orientation = rotation @ orientation
        elif cmd == cmd_anticlockwise:
            turn_radius = trace_data.cmd_param_dist[l]
            angle = np.radians(trace_data.cmd_param_angle[l])
            rotation = np.array([
                [np.cos(angle), -np.sin(angle)],
                [np.sin(angle), np.cos(angle)]
            ])
            next_loc = orientation * turn_radius
            next_loc += rotation @ orientation * turn_radius + loc
            orientation = rotation @ orientation

        ax.plot([loc[0], next_loc[0]], [loc[1], next_loc[1]], c='blue')
        loc = next_loc



    # Format plot
    plt.title('Position')
    plt.legend()
    plt.ylabel('Y')
    plt.xlabel('X')
    ax.set_xlim([-frame_size, frame_size])
    ax.set_ylim([-frame_size, frame_size])


# Called by multiprocess to start the animation
def start_animation(shared_dict, num_markers, trace_data, route_locations, starting_orientation):
    xs = []
    ys = []
    for _ in range(num_markers):
        xs.append([])
        ys.append([])
    ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys, shared_dict, num_markers, trace_data, route_locations, starting_orientation), interval=1)
    plt.show()
