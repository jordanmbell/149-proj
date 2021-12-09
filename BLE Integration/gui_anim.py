
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Set up graph
frame_size = 5
plt.style.use('seaborn')
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(1, 1, 1)
xs = [[],[],[],[]]
ys = [[],[],[],[]]
ax.set_xlim([-frame_size, frame_size])
ax.set_ylim([-frame_size, frame_size])


# This function is called periodically from FuncAnimation
# xs and ys are lists of past x,y to show a trail
def animate(i, xs, ys, shared_dict):

    # Add x and y to lists
    #print(shared_dict)
    for k in range(4):
        x, y = shared_dict[k]
        xs[k].append(x)
        ys[k].append(y)

    # Limit x and y lists to n_items
    n_items = 20
    xs = xs[-n_items:]
    ys = ys[-n_items:]


    # Draw x and y lists
    ax.clear()
    n_div = 20
    colors = ['blue', 'red', 'orange', 'green']
    for k in range(4):
        for j in range(n_div):
            x_range, y_range = xs[k][-(n_items - (n_items//n_div)*j):], ys[k][-(n_items - (n_items//n_div)*j):]
            ax.plot(x_range, y_range, alpha=0.1*j, c=colors[k] )

    for k in range(4):
        ax.scatter(xs[k][-1], ys[k][-1], c=colors[k], s=20, label=str(k))

    # Format plot
    plt.title('Position')
    plt.legend()
    plt.ylabel('Y')
    plt.xlabel('X')
    ax.set_xlim([-frame_size, frame_size])
    ax.set_ylim([-frame_size, frame_size])

# Called by multiprocess to start the animation
def start_animation(shared_dict):
    ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys, shared_dict), interval=1)
    plt.show()
