import random
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

from Configuration import config
from Vehicle import State, Vehicle
from simulation import adaptive_seek
import numpy as np
from Methods import *

def graph_x_vs_t(simulation_step:np.ndarray, positions_history:np.ndarray, velocities_history:np.ndarray):
    """Plot positions of all cars over time"""
    plt.figure(figsize=(10, 6))
    for vid in range(0, config["num_vehicles"]):
        if vid == 0:
             plt.scatter(
                simulation_step,
                positions_history[:, vid],
                c='red',          
                s=5,                          
                )           
        else:
            #TODO: Fix data types. We shouldn't have to convert a list of lists into an array here
            #TODO: Color gradient is across the velocities of each individual car, rather than across
            # a shared velocity as a whole. For example, car 1 could go from 0 to 1 in the full spectrum,
            # and car 2 could go from 0 to 100 and have the same color scheme. 
            velocities_history = np.array(velocities_history)
            positions_history = np.array(positions_history)
            color_gradient = velocities_history[:, vid]
            colors = ["#00008B", "#90EE90"]  # Dark blue to light green
            cmap = mcolors.LinearSegmentedColormap.from_list("blue_green", colors)
            # plt.scatter(simulation_step, positions_history[:, vid], label=f'Car {vid}', s=5)
            # plt.plot(simulation_step, positions_history[:, vid], 'b')
            plt.scatter(
                simulation_step,
                positions_history[:, vid],
                c=color_gradient,  
                cmap=cmap,                
                s=5,                          
            )
    plt.xlabel('Simulation Step')
    plt.ylabel('Position (state.x)')
    plt.title('Car Positions Over Time')
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()

#TODO: Make it so we can choose which graphs to show

def graph_stats(simulation_step, stats):
    """graph all stats"""
    plt.figure(figsize=(10, 6))
    for key in stats:
        # plt.scatter(simulation_step, stats[f"{key}"], label=f"{key}", s=10)
        plt.plot(simulation_step, stats[f"{key}"], label=f"{key}")

    plt.xlabel('Simulation Step')
    plt.ylabel('Speed Range')
    plt.title('Speed Range Over Time')
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()      

