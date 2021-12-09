
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Set up graph
frame_size = 5
plt.style.use('seaborn')
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(projection='3d')
ax.set_xlim([-frame_size, frame_size])
ax.set_ylim([-frame_size, frame_size])
ax.set_zlim([-frame_size, frame_size])
colors = ['blue', 'orange', 'green', 'red', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']


# This function is called periodically from FuncAnimation
def animate(i, unused, unused2, shared_dict, num_markers):
    # Draw x and y and z
    ax.clear()
    n_div = 20

    for k in range(num_markers):
        x, y, z = shared_dict[k]
        ax.scatter(x, y, z, c=colors[k], s=20, label=str(k))

    # Add plane for 0
    xx, yy = np.meshgrid(np.linspace(-frame_size, frame_size), np.linspace(-frame_size, frame_size))
    normal = np.array([0, 0, 1])
    z = (-normal[0] * xx - normal[1] * yy) * 1. /normal[2]
    ax.plot_surface(xx, yy, z, alpha=0.2)

    # Format plot
    plt.title('Position')
    plt.legend(loc='upper right')
    ax.set_zlabel('Z')
    ax.set_ylabel('Y')
    ax.set_xlabel('X')
    ax.set_xlim([-frame_size, frame_size])
    ax.set_ylim([-frame_size, frame_size])
    ax.set_zlim([-frame_size, frame_size])
    ax.view_init(20, 60)


# Called by multiprocess to start the animation
def start_animation(shared_dict, num_markers):
    xs = []
    ys = [] 
    ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys, shared_dict, num_markers), interval=1)
    plt.show()

