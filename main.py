import simulation
import animate
import pickle

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

from Configuration import config
from Vehicle import State, Vehicle
import simulation
import numpy as np
from Methods import *

def save_data(self, filepath):         
    with open(filepath, 'wb') as f:
        pickle.dump(self.data, f)

def load_data(self, filepath):
    with open(filepath, 'rb') as f:
        return pickle.load(f)
    
def graph_statistics(data:simulation.SimulationData):
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

def graph_x_vs_t(data:simulation.SimulationData):
    """Plot positions of all cars over time"""
    plt.figure(figsize=(10, 6))

    # Color Map Setup
    #TODO: Color gradient is across the velocities of each individual car, rather than across
    # a shared velocity as a whole. For example, car 1 could go from 0 to 1 in the full spectrum of colors,
    # and car 2 could go from 0 to 100 and have the same color scheme. 
    norm = mcolors.Normalize(vmin = np.min(data.velocity), vmax = np.max(data.velocity))
    norm = mcolors.Normalize(vmin = 0, vmax = np.max(data.velocity))
    colors = ["#0101C36C", "#90EE90"]  # Dark blue to light green
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
            plt.scatter(
                data.simulation_step,
                data.position[:, vid],
                c=data.velocity[:, vid],  
                norm=norm,
                cmap=cmap,                
                s=1,                          
            )
    plt.xlabel('Simulation Step')
    plt.ylabel('Position (state.x)')
    plt.title('Car Positions Over Time')
    plt.colorbar()
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()

def graph_all(data):
    fig, axs = plt.subplots(1, 2, figsize=(14, 6))

    # -----------------------------------------------------------
    # Left plot: positions over time
    # -----------------------------------------------------------
    ax = axs[0]

    norm = mcolors.Normalize(vmin=0, vmax=np.max(data.velocity))
    colors = ["#0101C36C", "#90EE90"]
    cmap = mcolors.LinearSegmentedColormap.from_list("blue_green", colors)

    for vid in range(config["num_vehicles"]):
        if vid == 0:
            ax.scatter(
                data.simulation_step,
                data.position[:, vid],
                c="red",
                s=5
            )
        else:
            ax.scatter(
                data.simulation_step,
                data.position[:, vid],
                c=data.velocity[:, vid],
                norm=norm,
                cmap=cmap,
                s=1
            )

    ax.set_xlabel("Simulation Step")
    ax.set_ylabel("Position (state.x)")
    ax.set_title("Car Positions Over Time")
    ax.grid()
    ax.legend()
    fig.colorbar(
        plt.cm.ScalarMappable(norm=norm, cmap=cmap),
        ax=ax,
        fraction=0.046,
        pad=0.04
    )

    # -----------------------------------------------------------
    # Right plot: speed stats
    # -----------------------------------------------------------
    ax = axs[1]

    ax.plot(data.simulation_step, data.max_speed, label="Max Speed")
    ax.plot(data.simulation_step, data.min_speed, label="Min Speed")
    ax.plot(data.simulation_step, data.average_speed, label="Average Speed")
    ax.plot(data.simulation_step, data.range, label="Range")

    ax.set_xlabel("Simulation Step")
    ax.set_ylabel("Velocity")
    ax.set_title("Speed Ranges Over Time")
    ax.grid()
    ax.legend()

    # -----------------------------------------------------------
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    filepath = "./data.pkl"
    data = simulation.run_simulation()
    graph_all(data)
