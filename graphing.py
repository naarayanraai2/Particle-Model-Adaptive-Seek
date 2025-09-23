import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

from Configuration import config
from Vehicle import State, Vehicle
import simulation
import numpy as np
from Methods import *

def graph_x_vs_t(data:simulation.SimulationData):
    """Plot positions of all cars over time"""
    plt.figure(figsize=(10, 6))

    # Color Map Setup
    norm = mcolors.Normalize(vmin = np.min(data.velocity), vmax = np.max(data.velocity))
    colors = ["#0000AD", "#90EE90"]  # Dark blue to light green
    cmap = mcolors.LinearSegmentedColormap.from_list("blue_green", colors)

    #TODO: Modify SimulationData class to include num_vehicles
    for vid in range(0, config["num_vehicles"]):
        if vid == 0: # Car 0 Trajectory - Red 
             plt.scatter(
                data.simulation_step,
                data.position[:, vid],
                c='red',          
                s=5,                          
                )           
        else: # Other Car Trajectories - Velocity based color
            #TODO: Color gradient is across the velocities of each individual car, rather than across
            # a shared velocity as a whole. For example, car 1 could go from 0 to 1 in the full spectrum of colors,
            # and car 2 could go from 0 to 100 and have the same color scheme. 
            plt.scatter(
                data.simulation_step,
                data.position[:, vid],
                c=data.velocity[:, vid],  
                norm=norm,
                cmap=cmap,                
                s=5,                          
            )
    plt.xlabel('Simulation Step')
    plt.ylabel('Position (state.x)')
    plt.title('Car Positions Over Time')
    plt.colorbar()
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()

#TODO: Make it so we can choose which graphs to show

def graph_statistics(data:simulation.SimulationData):
    """graph all stats"""
    plt.figure(figsize=(10, 6))
    plt.plot(data.simulation_step, data.max_speed, label=f"Max Speed")
    plt.plot(data.simulation_step, data.min_speed, label=f"Min Speed")
    plt.plot(data.simulation_step, data.average_speed, label=f"Average Speed")
    plt.plot(data.simulation_step, data.range, label=f"Range")

    plt.xlabel('Simulation Step')
    plt.ylabel('Velocity')
    plt.title('Speed Ranges Over Time')
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()      

