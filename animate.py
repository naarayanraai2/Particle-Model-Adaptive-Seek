import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.colors as mcolors
import simulation
import numpy as np
from Configuration import config

import os
import json

def circular(data:simulation.SimulationData):
    '''
    Simulation of positions mapped onto a 2D Circle with velocity colormap
    '''
    framerate = 60

    fig = plt.figure(figsize=(6,10))

    norm = mcolors.Normalize(vmin = 0, vmax = np.max(data.velocity))
    colors = ["#0101C36C", "#90EE90"]  # Dark blue to light green
    cmap = mcolors.LinearSegmentedColormap.from_list("blue_green", colors)

    radius = config["radius"]
    # Assumes that 0 distance = (r, 0) on the xy plane
    time = data.simulation_step
    # Convert linear path to circular path
    x_pos = radius * np.cos(data.position / radius)
    y_pos = radius * np.sin(data.position / radius)

    coords = np.stack([x_pos[0], y_pos[0]], axis=1)  # shape (number of cars, 2)
    points = plt.scatter(coords[:, 0], coords[:, 1], c="k", s=80)

    def update(frame):
        coords = np.stack([x_pos[frame], y_pos[frame]], axis=1)  # shape (number of cars, 2)
        points.set_offsets(coords) # set the new scatter positions
        # take velocity column at time point, normalize value, then convert to RGB with colormap, and set color for all points
        points.set_color(cmap(norm(data.velocity[frame,:]))) # take velocity column at time point, normalize value, then convert to RGB with colormap, and set color for all points
        return (points,) # MUST return a tuple, so func is an iterable

    ani = animation.FuncAnimation(
        fig=fig,
        func=update,
        frames=config["simulation_steps"],
        interval=1/framerate*1000,
        blit=True
    )

    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Adaptive Seek Model for Circular Path")
    plt.grid()
    plt.show()